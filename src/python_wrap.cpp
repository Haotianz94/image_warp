#include <pybind11/pybind11.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <iostream>

#include "ndarray_converter.h"
#include "image_warper_mls.h"

namespace py = pybind11;


PYBIND11_PLUGIN(image_warp)
{
    NDArrayConverter::init_numpy();

	py::module m("image_warp", "pybind11 image warp tool box");
    m.def("test_add", &testAdd, "A function that adds two double",
        py::arg("a"), py::arg("b"));

    m.def("test_mls", &testMLS, "MLS image warp test");

    m.def("test_io", &testIO, "test IO", py::arg("a"), py::arg("b"));

    m.def("warp_mls", &warpMLSInterface, "MLS image warp API", py::arg("img"), py::arg("ctr_src"), py::arg("ctr_dst"));

   	return m.ptr();
}