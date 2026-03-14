---
applyTo: "src/reconstruction/**,src/measurement/**,src/pointcloud/**"
---

# 算法开发智能体指令

> 适用范围：`src/reconstruction/`、`src/measurement/`、`src/pointcloud/`

## 翻译质量标准

- 数值结果与 MATLAB 原版一致（浮点误差 < 1e-6）
- 所有公共函数入口必须检查输入 `cv::Mat` 是否为空（`.empty()` → 抛异常）
- 不使用任何全局可变状态（算法函数必须是可重入的）
- 禁止在算法层包含任何 Qt 头文件（`QString`、`QImage` 等）

## MATLAB → C++ 数据类型对照

| MATLAB | C++ |
|--------|-----|
| `double` 矩阵 | `cv::Mat`（type = `CV_64F`） |
| `single` 矩阵 | `cv::Mat`（type = `CV_32F`） |
| `uint8` 图像 | `cv::Mat`（type = `CV_8U`） |
| `logical` 掩膜 | `cv::Mat`（type = `CV_8U`，值 0/255） |
| `cell{i,j}` | `std::vector<std::vector<cv::Mat>>` 或 `std::array` |
| 标量 `double` | `double` |
| 标量 `int` | `int` |

## MATLAB → C++ 操作对照

| MATLAB 操作 | C++ 替代 |
|------------|---------|
| `atan2(y, x)` 逐元素 | `cv::phase(x, y, out)` 或逐像素 `std::atan2f` |
| `A .* B` | `cv::multiply(A, B, C)` |
| `A ./ B` | `cv::divide(A, B, C)` |
| `A .^ n` | `cv::pow(A, n, C)` |
| `sqrt(A)` | `cv::sqrt(A, C)` |
| `floor(x)` 标量 | `static_cast<int>(std::floor(x))` |
| `sum(A(:))` | `cv::sum(A)[0]` |
| `gradient(A)` | `cv::Sobel(A, out, -1, 1, 0)` / `cv::Sobel(A, out, -1, 0, 1)` |
| `norm(v)` | `cv::norm(v)` |
| `imfilter(I, h, 'replicate')` | `cv::filter2D(I, out, -1, h, {-1,-1}, 0, cv::BORDER_REPLICATE)` |
| `fspecial('gaussian', ...)` | `cv::getGaussianKernel(...)` |
| `figure; imshow(I)` | **跳过**（UI 层负责显示） |
| `disp(msg)` | `Logger::instance().log(Logger::INFO, "Module", msg)` |

## 多线程实现规范

```cpp
// Pro1 与 Pro2 相位解码并行
auto futPro1 = pool.enqueue([&]{ return decoder_.decode(imgs1_); });
auto futPro2 = pool.enqueue([&]{ return decoder_.decode(imgs2_); });
auto phi1 = futPro1.get();
auto phi2 = futPro2.get();
// 极线修正：串行，依赖 phi1 + phi2
auto corrected = epiCorrector_.correct(phi1, phi2, params_);
```

- `cv::Mat` 跨线程传递前必须 `.clone()`
- `pcl::PointCloud::Ptr`（`shared_ptr`）传递时使用读写锁
- ThreadPool 实现在 `src/common/ThreadPool.h`（Agent3 提供）

## 接口规范

- 所有公共类继承 `src/common/interfaces.h` 中对应的纯虚基类
- 构造函数不执行耗时操作；耗时初始化放在 `bool init()` 函数，失败返回 `false`
- 错误用 `std::optional<T>` 返回无效值，或直接抛出 `AppException` 子类

## 点云输出规范

- 默认输出 `pcl::PointCloud<pcl::PointXYZRGB>` 带颜色（颜色映射高度 Jet）
- 默认保存格式：PLY（ASCII），支持 PCD / CSV
- 空点云（size = 0）不保存，抛出 `ReconstructException`

## 禁止事项

- 禁止 `cv::imshow` / `cv::waitKey`（不在算法层显示图像）
- 禁止写死任何路径字符串（路径通过参数传入）
- 禁止在算法函数内部直接调用 Qt 信号（通过回调接口转发）
- 禁止修改 `algorithm/` 目录下的 MATLAB 文件
