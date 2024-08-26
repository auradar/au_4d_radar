// automatically generated by the FlatBuffers compiler, do not modify


#ifndef FLATBUFFERS_GENERATED_HEARTBEAT_AU_H_
#define FLATBUFFERS_GENERATED_HEARTBEAT_AU_H_

#include "flatbuffers/flatbuffers.h"

// Ensure the included flatbuffers.h is the same version as when this file was
// generated, otherwise it may not be compatible.
static_assert(FLATBUFFERS_VERSION_MAJOR == 24 &&
              FLATBUFFERS_VERSION_MINOR == 3 &&
              FLATBUFFERS_VERSION_REVISION == 25,
             "Non-compatible flatbuffers version included");

namespace AU {

struct Heartbeat;
struct HeartbeatBuilder;

struct Heartbeat FLATBUFFERS_FINAL_CLASS : private ::flatbuffers::Table {
  typedef HeartbeatBuilder Builder;
  enum FlatBuffersVTableOffset FLATBUFFERS_VTABLE_UNDERLYING_TYPE {
    VT_MSSG_TYPE = 4,
    VT_CLIENT_HOSTNAME = 6,
    VT_STATUS = 8,
    VT_TIMESTAMP = 10
  };
  const ::flatbuffers::String *mssg_type() const {
    return GetPointer<const ::flatbuffers::String *>(VT_MSSG_TYPE);
  }
  const ::flatbuffers::String *client_hostname() const {
    return GetPointer<const ::flatbuffers::String *>(VT_CLIENT_HOSTNAME);
  }
  uint32_t status() const {
    return GetField<uint32_t>(VT_STATUS, 0);
  }
  uint64_t timestamp() const {
    return GetField<uint64_t>(VT_TIMESTAMP, 0);
  }
  bool Verify(::flatbuffers::Verifier &verifier) const {
    return VerifyTableStart(verifier) &&
           VerifyOffset(verifier, VT_MSSG_TYPE) &&
           verifier.VerifyString(mssg_type()) &&
           VerifyOffset(verifier, VT_CLIENT_HOSTNAME) &&
           verifier.VerifyString(client_hostname()) &&
           VerifyField<uint32_t>(verifier, VT_STATUS, 4) &&
           VerifyField<uint64_t>(verifier, VT_TIMESTAMP, 8) &&
           verifier.EndTable();
  }
};

struct HeartbeatBuilder {
  typedef Heartbeat Table;
  ::flatbuffers::FlatBufferBuilder &fbb_;
  ::flatbuffers::uoffset_t start_;
  void add_mssg_type(::flatbuffers::Offset<::flatbuffers::String> mssg_type) {
    fbb_.AddOffset(Heartbeat::VT_MSSG_TYPE, mssg_type);
  }
  void add_client_hostname(::flatbuffers::Offset<::flatbuffers::String> client_hostname) {
    fbb_.AddOffset(Heartbeat::VT_CLIENT_HOSTNAME, client_hostname);
  }
  void add_status(uint32_t status) {
    fbb_.AddElement<uint32_t>(Heartbeat::VT_STATUS, status, 0);
  }
  void add_timestamp(uint64_t timestamp) {
    fbb_.AddElement<uint64_t>(Heartbeat::VT_TIMESTAMP, timestamp, 0);
  }
  explicit HeartbeatBuilder(::flatbuffers::FlatBufferBuilder &_fbb)
        : fbb_(_fbb) {
    start_ = fbb_.StartTable();
  }
  ::flatbuffers::Offset<Heartbeat> Finish() {
    const auto end = fbb_.EndTable(start_);
    auto o = ::flatbuffers::Offset<Heartbeat>(end);
    return o;
  }
};

inline ::flatbuffers::Offset<Heartbeat> CreateHeartbeat(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    ::flatbuffers::Offset<::flatbuffers::String> mssg_type = 0,
    ::flatbuffers::Offset<::flatbuffers::String> client_hostname = 0,
    uint32_t status = 0,
    uint64_t timestamp = 0) {
  HeartbeatBuilder builder_(_fbb);
  builder_.add_timestamp(timestamp);
  builder_.add_status(status);
  builder_.add_client_hostname(client_hostname);
  builder_.add_mssg_type(mssg_type);
  return builder_.Finish();
}

inline ::flatbuffers::Offset<Heartbeat> CreateHeartbeatDirect(
    ::flatbuffers::FlatBufferBuilder &_fbb,
    const char *mssg_type = nullptr,
    const char *client_hostname = nullptr,
    uint32_t status = 0,
    uint64_t timestamp = 0) {
  auto mssg_type__ = mssg_type ? _fbb.CreateString(mssg_type) : 0;
  auto client_hostname__ = client_hostname ? _fbb.CreateString(client_hostname) : 0;
  return AU::CreateHeartbeat(
      _fbb,
      mssg_type__,
      client_hostname__,
      status,
      timestamp);
}

inline const AU::Heartbeat *GetHeartbeat(const void *buf) {
  return ::flatbuffers::GetRoot<AU::Heartbeat>(buf);
}

inline const AU::Heartbeat *GetSizePrefixedHeartbeat(const void *buf) {
  return ::flatbuffers::GetSizePrefixedRoot<AU::Heartbeat>(buf);
}

inline bool VerifyHeartbeatBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifyBuffer<AU::Heartbeat>(nullptr);
}

inline bool VerifySizePrefixedHeartbeatBuffer(
    ::flatbuffers::Verifier &verifier) {
  return verifier.VerifySizePrefixedBuffer<AU::Heartbeat>(nullptr);
}

inline void FinishHeartbeatBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<AU::Heartbeat> root) {
  fbb.Finish(root);
}

inline void FinishSizePrefixedHeartbeatBuffer(
    ::flatbuffers::FlatBufferBuilder &fbb,
    ::flatbuffers::Offset<AU::Heartbeat> root) {
  fbb.FinishSizePrefixed(root);
}

}  // namespace AU

#endif  // FLATBUFFERS_GENERATED_HEARTBEAT_AU_H_