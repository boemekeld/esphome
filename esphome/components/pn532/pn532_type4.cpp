#include "pn532.h"
#include "esphome/core/log.h"
#include <algorithm>

namespace esphome {
namespace pn532 {

static const char *const TAG_TYPE4 = "pn532.type4";

// Helper: wrap INDATAEXCHANGE + check SW1/SW2 = 0x90/0x00
bool PN532::exchange_apdu_(const std::vector<uint8_t> &apdu,
                           std::vector<uint8_t> &response) {
  // [D4][40] = INDATAEXCHANGE, [01] = target #1, [00] = ISO-DEP framing
  std::vector<uint8_t> cmd = {PN532_COMMAND_INDATAEXCHANGE, 0x01, 0x00};
  cmd.insert(cmd.end(), apdu.begin(), apdu.end());
  if (!this->write_command_(cmd)) {
    ESP_LOGE(TAG_TYPE4, "Failed to send APDU");
    return false;
  }
  std::vector<uint8_t> data;
  if (!this->read_response(PN532_COMMAND_INDATAEXCHANGE, data) ||
      data.size() < 3 || data[0] != 0x00) {
    ESP_LOGE(TAG_TYPE4, "No resp or status byte != 0");
    return false;
  }
  // Last two bytes are SW1 SW2
  uint8_t sw1 = data[data.size() - 2], sw2 = data.back();
  if (sw1 != 0x90 || sw2 != 0x00) {
    ESP_LOGE(TAG_TYPE4, "APDU error SW1/2=%02X %02X", sw1, sw2);
    return false;
  }
  // Strip status byte + SW1/SW2, return only payload
  response.assign(data.begin() + 1, data.end() - 2);
  return true;
}

// Main Type 4 read sequence
std::unique_ptr<nfc::NfcTag> PN532::read_iso_dep_tag_(
    const std::vector<uint8_t> &inlist, const std::vector<uint8_t> &uid) {
  std::vector<uint8_t> apdu, resp;

  // 1) Select NDEF application AID D2760000850101
  apdu = {0x00, 0xA4, 0x04, 0x00, 0x07,
          0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01, 0x00};
  if (!this->exchange_apdu_(apdu, resp)) {
    ESP_LOGE(TAG_TYPE4, "Select NDEF AID failed");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }

  // 2) Select CC file (ID = E1 03)
  apdu = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03, 0x00};
  if (!this->exchange_apdu_(apdu, resp)) {
    ESP_LOGE(TAG_TYPE4, "Select CC file failed");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }

  // 3) Read first 15 bytes of CC
  apdu = {0x00, 0xB0, 0x00, 0x00, 0x0F};
  if (!this->exchange_apdu_(apdu, resp) || resp.size() < 4) {
    ESP_LOGE(TAG_TYPE4, "Read CC file failed");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }

  // Parse TLV in CC to find NDEF File Control (tag=0x04)
  uint16_t ndef_file_id = 0, ndef_max_len = 0;
  size_t idx = 2;  // skip CCLEN + version
  while (idx + 3 < resp.size()) {
    uint8_t t = resp[idx], l = resp[idx + 1];
    if (t == 0x04 && l >= 4) {
      ndef_file_id  = (resp[idx + 2] << 8) | resp[idx + 3];
      ndef_max_len  = (resp[idx + 4] << 8) | resp[idx + 5];
      break;
    }
    idx += 2 + l;
  }
  if (!ndef_file_id) {
    ESP_LOGE(TAG_TYPE4, "No NDEF file control TLV");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }

  // 4) Select the NDEF file
  apdu = {0x00, 0xA4, 0x00, 0x0C, 0x02,
          uint8_t(ndef_file_id >> 8), uint8_t(ndef_file_id & 0xFF), 0x00};
  if (!this->exchange_apdu_(apdu, resp)) {
    ESP_LOGE(TAG_TYPE4, "Select NDEF file failed");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }

  // 5) Read NLEN (2-byte length)
  apdu = {0x00, 0xB0, 0x00, 0x00, 0x02};
  if (!this->exchange_apdu_(apdu, resp) || resp.size() < 2) {
    ESP_LOGE(TAG_TYPE4, "Read NDEF length failed");
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);
  }
  uint16_t ndef_len = (resp[0] << 8) | resp[1];
  if (!ndef_len)
    return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4);

  // 6) Read the NDEF message in chunks up to 0xFF bytes
  std::vector<uint8_t> ndef_data;
  size_t offset = 2;
  while (ndef_data.size() < ndef_len) {
    uint8_t chunk = uint8_t(std::min<size_t>(ndef_len - ndef_data.size(), 0xFF));
    uint8_t p1 = (offset >> 8) & 0xFF, p2 = offset & 0xFF;
    apdu = {0x00, 0xB0, p1, p2, chunk};
    if (!this->exchange_apdu_(apdu, resp)) {
      ESP_LOGE(TAG_TYPE4, "Read NDEF data failed @%u", offset);
      break;
    }
    ndef_data.insert(ndef_data.end(), resp.begin(), resp.begin() + chunk);
    offset += chunk;
  }

  return make_unique<nfc::NfcTag>(uid, nfc::NFC_FORUM_TYPE_4, ndef_data);
}

}  // namespace pn532
}  // namespace esphome