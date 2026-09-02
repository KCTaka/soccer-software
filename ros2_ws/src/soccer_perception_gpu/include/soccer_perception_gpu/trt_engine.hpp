// Minimal RAII wrapper around a serialised TensorRT engine.
//
// Deliberately does NOT depend on Isaac ROS / NITROS: we need exactly one
// feature (run an engine on a stream) and NITROS pulls in the GXF runtime.
// See docs/architecture/perception_gpu_migration.md §4.
#ifndef SOCCER_PERCEPTION_GPU__TRT_ENGINE_HPP_
#define SOCCER_PERCEPTION_GPU__TRT_ENGINE_HPP_

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace nvinfer1
{
class ICudaEngine;
class IExecutionContext;
class IRuntime;
class ILogger;
}  // namespace nvinfer1

namespace soccer_perception_gpu
{

struct TensorInfo
{
  std::string name;
  std::vector<int64_t> shape;
  size_t elements = 0;   // product of shape
  size_t bytes = 0;      // elements * sizeof(float); engines here are all fp32 I/O
  bool is_input = false;
};

/// Loads a `.engine` file, owns its device buffers, and runs synchronous
/// inference on a private CUDA stream.
///
/// Not thread-safe: one instance per node, called from one callback.
class TrtEngine
{
public:
  /// @throws std::runtime_error if the file is missing or fails to deserialise.
  explicit TrtEngine(const std::string & engine_path);
  ~TrtEngine();

  TrtEngine(const TrtEngine &) = delete;
  TrtEngine & operator=(const TrtEngine &) = delete;

  /// Copy `host` into the named input device buffer.
  void set_input(const std::string & name, const float * host, size_t elements);

  /// Run the engine and block until the stream is drained.
  void infer();

  /// Host-side view of a named output, valid until the next `infer()`.
  const std::vector<float> & output(const std::string & name) const;

  const std::vector<TensorInfo> & tensors() const {return tensors_;}
  const TensorInfo & tensor(const std::string & name) const;

  /// TensorRT runtime version, e.g. 101602 for 10.16.2.
  static int runtime_version();
  /// Version the caller was compiled against, for the mismatch check.
  static int compiled_version();

private:
  std::unique_ptr<nvinfer1::ILogger> logger_;
  nvinfer1::IRuntime * runtime_ = nullptr;
  nvinfer1::ICudaEngine * engine_ = nullptr;
  nvinfer1::IExecutionContext * context_ = nullptr;
  void * stream_ = nullptr;  // cudaStream_t

  std::vector<TensorInfo> tensors_;
  std::unordered_map<std::string, void *> device_;
  std::unordered_map<std::string, std::vector<float>> host_out_;
};

}  // namespace soccer_perception_gpu

#endif  // SOCCER_PERCEPTION_GPU__TRT_ENGINE_HPP_
