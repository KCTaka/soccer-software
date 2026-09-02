#include "soccer_perception_gpu/trt_engine.hpp"

#include <NvInfer.h>
#include <NvInferVersion.h>
#include <cuda_runtime.h>

#include <cstdio>
#include <fstream>
#include <numeric>
#include <sstream>

namespace soccer_perception_gpu
{
namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::fprintf(stderr, "[TensorRT] %s\n", msg);
    }
  }
};

void cuda_check(cudaError_t err, const char * what)
{
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
  }
}

}  // namespace

TrtEngine::TrtEngine(const std::string & engine_path)
: logger_(std::make_unique<TrtLogger>())
{
  std::ifstream f(engine_path, std::ios::binary | std::ios::ate);
  if (!f) {
    throw std::runtime_error("cannot open TensorRT engine: " + engine_path);
  }
  const std::streamsize size = f.tellg();
  f.seekg(0, std::ios::beg);
  std::vector<char> blob(static_cast<size_t>(size));
  if (!f.read(blob.data(), size)) {
    throw std::runtime_error("short read on TensorRT engine: " + engine_path);
  }

  runtime_ = nvinfer1::createInferRuntime(*logger_);
  if (!runtime_) {throw std::runtime_error("createInferRuntime failed");}

  engine_ = runtime_->deserializeCudaEngine(blob.data(), blob.size());
  if (!engine_) {
    // Nearly always a version mismatch between the builder and this runtime.
    std::ostringstream os;
    os << "deserializeCudaEngine failed for " << engine_path
       << " (runtime " << runtime_version() << ", headers " << compiled_version()
       << ") - rebuild the engine with the trtexec that matches this runtime";
    throw std::runtime_error(os.str());
  }

  context_ = engine_->createExecutionContext();
  if (!context_) {throw std::runtime_error("createExecutionContext failed");}

  cudaStream_t s = nullptr;
  cuda_check(cudaStreamCreate(&s), "cudaStreamCreate");
  stream_ = s;

  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char * name = engine_->getIOTensorName(i);
    const auto dims = engine_->getTensorShape(name);

    TensorInfo info;
    info.name = name;
    info.is_input =
      engine_->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT;
    info.elements = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      if (dims.d[d] < 0) {
        throw std::runtime_error(
                std::string("dynamic shape not supported for tensor ") + name +
                "; build the engine with a fixed shape");
      }
      info.shape.push_back(dims.d[d]);
      info.elements *= static_cast<size_t>(dims.d[d]);
    }
    info.bytes = info.elements * sizeof(float);

    void * dev = nullptr;
    cuda_check(cudaMalloc(&dev, info.bytes), "cudaMalloc");
    device_[info.name] = dev;
    context_->setTensorAddress(name, dev);

    if (!info.is_input) {
      host_out_[info.name].resize(info.elements);
    }
    tensors_.push_back(std::move(info));
  }
}

TrtEngine::~TrtEngine()
{
  for (auto & [name, ptr] : device_) {
    (void)name;
    cudaFree(ptr);
  }
  if (stream_) {cudaStreamDestroy(static_cast<cudaStream_t>(stream_));}
  delete context_;
  delete engine_;
  delete runtime_;
}

const TensorInfo & TrtEngine::tensor(const std::string & name) const
{
  for (const auto & t : tensors_) {
    if (t.name == name) {return t;}
  }
  throw std::runtime_error("no such tensor: " + name);
}

void TrtEngine::set_input(const std::string & name, const float * host, size_t elements)
{
  const TensorInfo & t = tensor(name);
  if (elements != t.elements) {
    throw std::runtime_error(
            "input size mismatch for " + name + ": got " + std::to_string(elements) +
            ", engine wants " + std::to_string(t.elements));
  }
  cuda_check(
    cudaMemcpyAsync(
      device_.at(name), host, t.bytes, cudaMemcpyHostToDevice,
      static_cast<cudaStream_t>(stream_)),
    "cudaMemcpyAsync H2D");
}

void TrtEngine::infer()
{
  if (!context_->enqueueV3(static_cast<cudaStream_t>(stream_))) {
    throw std::runtime_error("enqueueV3 failed");
  }
  for (const auto & t : tensors_) {
    if (t.is_input) {continue;}
    cuda_check(
      cudaMemcpyAsync(
        host_out_.at(t.name).data(), device_.at(t.name), t.bytes,
        cudaMemcpyDeviceToHost, static_cast<cudaStream_t>(stream_)),
      "cudaMemcpyAsync D2H");
  }
  cuda_check(cudaStreamSynchronize(static_cast<cudaStream_t>(stream_)), "cudaStreamSynchronize");
}

const std::vector<float> & TrtEngine::output(const std::string & name) const
{
  auto it = host_out_.find(name);
  if (it == host_out_.end()) {throw std::runtime_error("no such output: " + name);}
  return it->second;
}

int TrtEngine::runtime_version() {return getInferLibVersion();}
int TrtEngine::compiled_version() {return NV_TENSORRT_VERSION;}

}  // namespace soccer_perception_gpu
