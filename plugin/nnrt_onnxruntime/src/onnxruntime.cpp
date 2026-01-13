#include <algorithm>
#include <numeric>

#include <stepit/nnrt/onnxruntime.h>

namespace stepit {
OnnxrtApi::OnnxrtApi(const std::string &path, const YAML::Node &config) : NnrtApi(path, config) {
  memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, path.c_str());
  Ort::SessionOptions opts;
  opts.SetInterOpNumThreads(1);
  opts.SetIntraOpNumThreads(1);
  core_    = std::make_unique<Ort::Session>(env, path.c_str(), opts);
  num_in_  = core_->GetInputCount();
  num_out_ = core_->GetOutputCount();

  Ort::AllocatorWithDefaultOptions allocator;
  for (std::size_t i{}; i < num_in_; ++i) {
    auto shape   = getShapeFromTypeInfo(core_->GetInputTypeInfo(i));
    int64_t size = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<>());
    auto name    = core_->GetInputNameAllocated(i, allocator);

    in_shapes_.emplace_back(std::move(shape));
    in_sizes_.push_back(size);
    in_names_.emplace_back(name.get());
    in_data_.emplace_back(size, 0.0F);
    inputs_.push_back(createTensor(in_data_[i].data(), size, in_shapes_[i]));
  }

  for (std::size_t i{}; i < num_out_; ++i) {
    auto shape   = getShapeFromTypeInfo(core_->GetOutputTypeInfo(i));
    int64_t size = std::accumulate(shape.begin(), shape.end(), 1, std::multiplies<>());
    auto name    = core_->GetOutputNameAllocated(i, allocator);

    out_shapes_.emplace_back(std::move(shape));
    out_data_.emplace_back(size, 0.0F);
    out_sizes_.push_back(size);
    out_names_.emplace_back(name.get());
    outputs_.push_back(createTensor(out_data_[i].data(), size, out_shapes_[i]));
  }

  postInit();
}

void OnnxrtApi::runInference() {
  std::vector<const char *> in_names, out_names;
  for (const auto &name : in_names_) in_names.emplace_back(name.data());
  for (const auto &name : out_names_) out_names.emplace_back(name.data());
  core_->Run(run_options_, in_names.data(), inputs_.data(), inputs_.size(), out_names.data(), outputs_.data(),
             outputs_.size());
  for (const auto &pair : recur_param_indices_) {
    std::copy_n(out_data_[pair.second].data(), in_sizes_[pair.first], in_data_[pair.first].data());
  }
}

void OnnxrtApi::clearState() {
  for (const auto &pair : recur_param_indices_) {
    std::fill(in_data_[pair.first].begin(), in_data_[pair.first].end(), 0.0F);
  }
}

void OnnxrtApi::setInput(std::size_t idx, float *data) {  // copy data to input buffer
  std::copy_n(data, in_sizes_[idx], in_data_[idx].data());
}

const float *OnnxrtApi::getOutput(std::size_t idx) { return out_data_[idx].data(); }

std::vector<int64_t> OnnxrtApi::getShapeFromTypeInfo(const Ort::TypeInfo &type_info) {
  auto info  = type_info.GetTensorTypeAndShapeInfo();
  auto shape = info.GetShape();
  for (auto &dim : shape) {
    if (dim == -1) dim = 1;
  }
  return shape;
}

STEPIT_REGISTER_NNRTAPI(onnxruntime, kDefPriority, NnrtApi::makeDerived<OnnxrtApi>);
}  // namespace stepit
