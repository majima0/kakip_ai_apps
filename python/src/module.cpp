#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <linux/drpai.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
/*DRP-AI TVM[*1] Runtime*/
#include "MeraDrpRuntimeWrapper.h"
/*Pre-processing Runtime Header*/
#include "PreRuntime.h"
/*Definition of Macros & other variables*/
#include "define.h"
/*box drawing*/
#include "box.h"
/*dmabuf for Pre-processing Runtime input data*/
#include "dmabuf.h"
#include <tuple>

#include "utils.h"

namespace py = pybind11;

// ライブラリのファイル名と同じにしないといけない
// ダブルクオーテーションで囲わない
PYBIND11_MODULE(pydrpai, m) {
    // cpp_extentino.__doc__に設定される
    m.doc() = "pybind11 example plugin"; // optional module docstring

    // クラスの定義はclass_<C++のクラス>(m, Pythonに公開するクラス名)で定義できる
    // Pythonに公開するクラス名はC++のクラス名と一致する必要はない
    // pybind11::init<>()はクラスのコンストラクタを呼び出す
    pybind11::class_<Box>(m, "Box")
        .def(pybind11::init<>())
        .def_readwrite("x", &Box::x)
        .def_readwrite("y", &Box::y)
        .def_readwrite("w", &Box::w)
        .def_readwrite("h", &Box::h);

    pybind11::class_<detection>(m, "detection")
        .def(pybind11::init<>())
        .def_readwrite("bbox", &detection::bbox)
        .def_readwrite("c", &detection::c)
        .def_readwrite("prob", &detection::prob);

    // pybind11ではポインターの型が使えないので整数型にキャストするuintptr_t
    // メンバの配列はdef_propertyでgetterとsetterを定義する。
    // 配列のポインタの値そのものは変更したくないのでsetterは何も指定しない。getterはpy::arrayを返すラムダ式を指定する。
    py::class_<s_preproc_param_t>(m, "s_preproc_param_t")
	.def(py::init<>())
	.def("get_ptr", [](s_preproc_param_t &self){return reinterpret_cast<uintptr_t>(&self); })
	.def_readwrite("pre_in_shape_w", &s_preproc_param_t::pre_in_shape_w)
	.def_readwrite("pre_in_shape_h", &s_preproc_param_t::pre_in_shape_h)
	.def_readwrite("pre_in_addr", &s_preproc_param_t::pre_in_addr)
	.def_readwrite("pre_in_format", &s_preproc_param_t::pre_in_format)
	.def_readwrite("pre_out_format", &s_preproc_param_t::pre_out_format)
	.def_readwrite("resize_alg", &s_preproc_param_t::resize_alg)
	.def_readwrite("resize_w", &s_preproc_param_t::resize_w)
	.def_readwrite("resize_h", &s_preproc_param_t::resize_h)
	.def_property("cof_add", [](s_preproc_param_t &self) -> py::array {
			auto dtype = py::dtype(py::format_descriptor<float>::format());
			auto base = py::array(dtype, {3}, {sizeof(float)});
			return py::array(dtype, {3}, {sizeof(float)}, self.cof_add, base);
			}, [](s_preproc_param_t &self) {})
	.def_property("cof_mul", [](s_preproc_param_t &self) -> py::array {
			auto dtype = py::dtype(py::format_descriptor<float>::format());
			auto base = py::array(dtype, {3}, {sizeof(float)});
			return py::array(dtype, {3}, {sizeof(float)}, self.cof_mul, base);
			}, [](s_preproc_param_t &self) {})
	.def_readwrite("crop_tl_x", &s_preproc_param_t::crop_tl_x)
	.def_readwrite("crop_tl_y", &s_preproc_param_t::crop_tl_y)
	.def_readwrite("crop_w", &s_preproc_param_t::crop_w)
	.def_readwrite("crop_h", &s_preproc_param_t::crop_h);

    // PreRuntimeのdrpai_obj_infoの値が初期化されていないので、初期化する必要がある
    pybind11::class_<PreRuntime>(m, "PreRuntime")
	.def(pybind11::init<>())
	.def("Load", [](PreRuntime& self, std::string pre_dir, uint64_t start_addr) { return self.Load(pre_dir, start_addr);})
	.def("Load", [](PreRuntime& self, std::string pre_dir, uint64_t start_addr = INVALID_ADDR, uint8_t mode = MODE_PRE) { return self.Load(pre_dir, start_addr, mode);}, py::arg("pre_dir"), py::arg("start_addr") = INVALID_ADDR, py::arg("mode") = MODE_PRE)
	.def("SetInput", &PreRuntime::SetInput)
	.def("Pre", [](PreRuntime& self, uintptr_t param) {
			s_preproc_param_t* param_buf = reinterpret_cast<s_preproc_param_t*>(param);
			printf("%hu %hu %lu\n", param_buf->pre_in_shape_w, param_buf->pre_in_shape_h, param_buf->pre_in_addr);
			void* output_ptr = malloc(sizeof(void*));
			memset(output_ptr, 0, sizeof(void*));
			uint32_t out_size;

			self.Pre(reinterpret_cast<s_preproc_param_t*>(param), &output_ptr, &out_size);
		       	return std::make_tuple(reinterpret_cast<uintptr_t>(output_ptr), out_size); })
	.def("Pre", [](PreRuntime& self, py::buffer out_ptr, py::buffer out_size, uint64_t phyaddr) {
			py::buffer_info out_ptr_info = out_ptr.request();
			py::buffer_info out_size_info = out_size.request();
			return self.Pre(static_cast<void**>(out_ptr_info.ptr), static_cast<uint32_t*>(out_size_info.ptr), phyaddr);})
	.def_readwrite("Occupied_size", &PreRuntime::Occupied_size);

    // 列挙型の定義はclass_<C++の列挙型>(m, Pythonに公開する列挙型)で定義できる
    // export_values()でcpp_extention.FLOAT32で使えるようになる
    pybind11::enum_<InOutDataType>(m, "InOutDataType")
	.value("FLOAT32", InOutDataType::FLOAT32)
	.value("FLOAT16", InOutDataType::FLOAT16)
	.value("OTHER", InOutDataType::OTHER)
	.export_values();


    // オーバーロードはできるが、uint64_tとuint32_tはPythonから見ればどちらもInt型なので使い方を分けることはできない
    pybind11::class_<MeraDrpRuntimeWrapper>(m, "MeraDrpRuntimeWrapper")
	.def(pybind11::init<>())
	.def("LoadModel", [](MeraDrpRuntimeWrapper& self, const std::string& model_dir, uint64_t start_address) { return self.LoadModel(model_dir, start_address); })
	.def("SetInput", [](MeraDrpRuntimeWrapper& self, int input_index, uintptr_t data_ptr) { return self.SetInput(input_index, (float*)data_ptr); })
	.def("Run", [](MeraDrpRuntimeWrapper& self) { self.Run(); })
	.def("Run", [](MeraDrpRuntimeWrapper& self, int freq_index) { self.Run(freq_index);})
	.def("ProfileRun", [](MeraDrpRuntimeWrapper& self, const std::string& profile_table, const std::string& profile_csv) { return self.ProfileRun(profile_table, profile_csv); })
	.def("ProfileRun", [](MeraDrpRuntimeWrapper& self, const std::string& profile_table, const std::string& profile_csv, int freq_index) { return self.ProfileRun(profile_table, profile_csv, freq_index); })
	.def("GetNumInput", &MeraDrpRuntimeWrapper::GetNumInput)
	.def("GetInputDataType", &MeraDrpRuntimeWrapper::GetInputDataType)
	.def("GetNumOutput", &MeraDrpRuntimeWrapper::GetNumOutput)
	.def("GetOutput",
			[](MeraDrpRuntimeWrapper& self, int index) {
				auto [type, ptr, size] = self.GetOutput(index);
				return std::make_tuple(type, py::capsule(ptr), size);
			});
	

    m.def("init_drpai", &init_drpai, "DrpAI init");

    pybind11::class_<dma_buffer>(m, "dma_buffer")
        .def(pybind11::init<>())
        .def_readwrite("idx", &dma_buffer::idx)
        .def_readwrite("dbuf_fd", &dma_buffer::dbuf_fd)
        .def_readwrite("size", &dma_buffer::size)
	.def_readwrite("phy_addr", &dma_buffer::phy_addr)
	.def_property("mem",
			[](dma_buffer& self){ return reinterpret_cast<uintptr_t>(self.mem);},
			[](dma_buffer& self, uintptr_t v) {self.mem = reinterpret_cast<void*>(v);});

    m.def("buffer_alloc_dmabuf",
		    [](int buf_size) {
		    	dma_buffer buffer;
		    	int8_t ret = buffer_alloc_dmabuf(&buffer, buf_size);
		    	return buffer;
		    });

    m.def("buffer_free_dmabuf", [](dma_buffer buffer){ buffer_free_dmabuf(&buffer); });

    m.def("buffer_flush_dmabuf", &buffer_flush_dmabuf);

    m.def("memcpy", [](uintptr_t buf1, uintptr_t buf2, int size){ 
		    memcpy(reinterpret_cast<void*>(buf1), reinterpret_cast<void*>(buf2), size); });

    m.def("get_result", &get_result);
}
