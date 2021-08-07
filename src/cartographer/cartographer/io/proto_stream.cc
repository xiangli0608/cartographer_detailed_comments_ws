/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/io/proto_stream.h"

#include "glog/logging.h"

namespace cartographer {
namespace io {

namespace {

// First eight bytes to identify our proto stream format.
const uint64 kMagic = 0x7b1d1f7b5bf501db;

// 写入8个字节的校验位
void WriteSizeAsLittleEndian(uint64 size, std::ostream* out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

// 读取前8个字节的值, 进行累加
bool ReadSizeAsLittleEndian(std::istream* in, uint64* size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

// 以二进制方式, 写入的方式打开文件, 并写入8个字节的数据校验
ProtoStreamWriter::ProtoStreamWriter(const std::string& filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  WriteSizeAsLittleEndian(kMagic, &out_);
}

// 将传入的数据先进行压缩, 再写入到文件中
void ProtoStreamWriter::Write(const std::string& uncompressed_data) {
  std::string compressed_data;
  // 对数据进行压缩
  common::FastGzipString(uncompressed_data, &compressed_data);
  // 根据数据的size写入文件
  WriteSizeAsLittleEndian(compressed_data.size(), &out_);
  // 将内存中 compressed_data 以二进制的形式写入文件
  out_.write(compressed_data.data(), compressed_data.size());
}

// 将数据写入文件中
void ProtoStreamWriter::WriteProto(const google::protobuf::Message& proto) {
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  // 压缩并写入
  Write(uncompressed_data);
}

// 关闭打开的文件
bool ProtoStreamWriter::Close() {
  out_.close();
  return !out_.fail();
}


// 读取pbstream文件, 并对前8个字节的数据进行校验
ProtoStreamReader::ProtoStreamReader(const std::string& filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64 magic;
  // 对前8个字节的数据进行校验
  if (!ReadSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  CHECK(in_.good()) << "Failed to open proto stream '" << filename << "'.";
}

// 读取数据并解压
bool ProtoStreamReader::Read(std::string* decompressed_data) {
  uint64 compressed_size;
  // 获取数据的size
  if (!ReadSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  // 根据size生成字符串
  std::string compressed_data(compressed_size, '\0');
  // 读取数据放入compressed_data中
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  // 进行解压
  common::FastGunzipString(compressed_data, decompressed_data);
  return true;
}

// 读取数据并返回protobuf格式的数据
bool ProtoStreamReader::ReadProto(google::protobuf::Message* proto) {
  std::string decompressed_data;
  return Read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoStreamReader::eof() const { return in_.eof(); }

}  // namespace io
}  // namespace cartographer
