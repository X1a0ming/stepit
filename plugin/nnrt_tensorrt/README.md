# nnrt_tensorrt

StepIt plugin for neural network inference on NVIDIA GPUs and Jetson platforms, e.g. Jetson Orin NX.

Provided factories:

- `stepit::NnrtApi`: `tensorrt`


### Tips: Optimize Performance on Jetson Platforms

```shell
sudo sysctl -w kernel.sched_rt_runtime_us=-1  # enable real-time scheduling
sudo jetson_clocks  # lock CPU and GPU clock frequency to maximum
```
