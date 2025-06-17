#include <algorithm>
#include <vector>
#include <memory>

#include "pn532.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pn532 {

static const char *const TAG_TYPE4 = "pn532.type4";
static const char *const NFC_FORUM_TYPE_4 = "NFC Forum Type 4";

// Member implementation of send_apdu_
bool PN532::send_apdu_(const std::vector<uint8_t> &apdu, std::vector<uint8_t> &response) {
  // Wrap in InDataExchange
  std::vector<uint8_t> data = {PN532_COMMAND_INDATAEXCHANGE, 0x01};
  data.insert(data.end(), apdu.begin(), apdu.end());

  if (!this->write_command_(data) ||
      !this->read_response(PN532_COMMAND_INDATAEXCHANGE, response) ||
      response.size() < 2) {
    return false;
  }
  // Check SW1SW2 == 0x90 0x00
  size_t len = response.size();
  uint8_t sw1 = response[len - 2], sw2 = response[len - 1];
  if (sw1 != 0x90 || sw2 != 0x00) {
    ESP_LOGW(TAG_TYPE4, "APDU error SW1SW2=%02X%02X", sw1, sw2);
    return false;
  }
  response.resize(len - 2);  // strip status bytes
  return true;
}

std::unique_ptr<nfc::NfcTag> PN532::read_type4_tag_(std::vector<uint8_t> &uid) {
  ESP_LOGD(TAG_TYPE4, "Reading Type 4 tag UID %s", nfc::format_uid(uid).c_str());
  std::vector<uint8_t> response;

  // 1) Select NDEF Application AID D2760000850101
  static constexpr uint8_t NDEF_AID[] = {0xD2, 0x76, 0x00, 0x00, 0x85, 0x01, 0x01};
  std::vector<uint8_t> apdu = {0x00, 0xA4, 0x04, 0x00, sizeof(NDEF_AID)};
  apdu.insert(apdu.end(), NDEF_AID, NDEF_AID + sizeof(NDEF_AID));
  apdu.push_back(0x00);
  if (!this->send_apdu_(apdu, response))
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);

  // 2) Select CC File (E1 03)
  apdu = {0x00, 0xA4, 0x00, 0x0C, 0x02, 0xE1, 0x03};
  if (!this->send_apdu_(apdu, response))
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);

  // 3) Read CC File
  apdu = {0x00, 0xB0, 0x00, 0x00, 0x00};
  if (!this->send_apdu_(apdu, response) || response.size() < 7)
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);

  // Parse CC TLV: after CCLEN(2B), MappingVer(1B), MLe(2B), MLc(2B)
  constexpr size_t CC_TLV_OFFSET = 2 + 1 + 2 + 2;  // ==7
  size_t idx = CC_TLV_OFFSET;
  if (response[idx] != 0x04 || response.size() < idx + 6) {
    ESP_LOGW(TAG_TYPE4, "Bad CC TLV");
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);
  }
  uint8_t file_hi = response[idx + 2], file_lo = response[idx + 3];
  uint16_t ndef_max = (response[idx + 4] << 8) | response[idx + 5];

  // 4) Select NDEF File (E1 04)
  apdu = {0x00, 0xA4, 0x00, 0x0C, 0x02, file_hi, file_lo};
  if (!this->send_apdu_(apdu, response))
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);

  // 5) Read full NDEF file in chunks
  std::vector<uint8_t> full_data;
  uint16_t offset = 0;
  while (offset < ndef_max) {
    uint8_t p1 = offset >> 8, p2 = offset & 0xFF;
    uint8_t le = std::min<uint16_t>(ndef_max - offset, 0xFF);
    apdu = {0x00, 0xB0, p1, p2, le};
    if (!this->send_apdu_(apdu, response)) break;
    full_data.insert(full_data.end(), response.begin(), response.end());
    offset += response.size();
    if (response.size() < le) break;
  }

  // 6) Extract NDEF TLV (0x03) and payload (short vs long)
  auto it = std::find(full_data.begin(), full_data.end(), 0x03);
  if (it == full_data.end()) {
    ESP_LOGW(TAG_TYPE4, "No NDEF TLV found");
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);
  }
  size_t base = it - full_data.begin();
  uint32_t length;
  size_t start;
  if (full_data[base + 1] != 0xFF) {
    length = full_data[base + 1];
    start = base + 2;
  } else {
    length = ((uint32_t)full_data[base + 2] << 8) | full_data[base + 3];
    start = base + 4;
  }
  if (start + length > full_data.size()) {
    ESP_LOGW(TAG_TYPE4, "NDEF data truncated");
    return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4);
  }
  std::vector<uint8_t> ndef_bytes(full_data.begin() + start, full_data.begin() + start + length);
  return make_unique<nfc::NfcTag>(uid, NFC_FORUM_TYPE_4, ndef_bytes);
}

bool PN532::write_type4_tag_(std::vector<uint8_t> &uid, nfc::NdefMessage *message) {
  ESP_LOGD(TAG_TYPE4, "Writing Type 4 tag UID %s", nfc::format_uid(uid).c_str());
  std::vector<uint8_t> response;

  // a) Select NDEF Application
  static constexpr uint8_t NDEF_AID[] = {0xD2,0x76,0x00,0x00,0x85,0x01,0x01};
  std::vector<uint8_t> apdu = {0x00,0xA4,0x04,0x00,sizeof(NDEF_AID)};
  apdu.insert(apdu.end(), NDEF_AID, NDEF_AID + sizeof(NDEF_AID));
  apdu.push_back(0x00);
  if (!this->send_apdu_(apdu, response)) return false;

  // b) Select NDEF File (E1 04)
  apdu = {0x00,0xA4,0x00,0x0C,0x02,0xE1,0x04};
  if (!this->send_apdu_(apdu, response)) return false;

  // c) Wrap NDEF payload in TLV
  auto body = message->encode();
  std::vector<uint8_t> tlv;
  tlv.push_back(0x03);
  if (body.size() < 0xFF) {
    tlv.push_back(body.size());
  } else {
    tlv.push_back(0xFF);
    tlv.push_back((body.size() >> 8) & 0xFF);
    tlv.push_back(body.size() & 0xFF);
  }
  tlv.insert(tlv.end(), body.begin(), body.end());
  tlv.push_back(0xFE);

  // d) UPDATE BINARY in 255-byte chunks
  size_t off = 0;
  while (off < tlv.size()) {
    uint8_t p1 = off >> 8, p2 = off & 0xFF;
    uint8_t lc = std::min<size_t>(tlv.size() - off, 0xFF);
    apdu = {0x00,0xD6,p1,p2,lc};
    apdu.insert(apdu.end(), tlv.begin() + off, tlv.begin() + off + lc);
    if (!this->send_apdu_(apdu, response)) {
      ESP_LOGE(TAG_TYPE4, "Failed UPDATE BINARY at offset %u", off);
      return false;
    }
    off += lc;
  }
  return true;
}

bool PN532::format_type4_tag_(std::vector<uint8_t> &/*uid*/) {
  ESP_LOGD(TAG_TYPE4, "Type 4 tags are preformatted—no action needed");
  return true;
}

}  // namespace pn532
}  // namespace esphome