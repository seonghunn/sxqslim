#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "MeshSimplify.h" // 경로가 올바른지 확인하세요.

namespace py = pybind11;
namespace qslim {

    PYBIND11_MODULE(QSlim, m) {
        py::class_<MeshSimplify>(m, "MeshSimplify")
                .def(py::init<Eigen::MatrixXd &, Eigen::MatrixXi &, double, std::string &>(),
                     py::arg("V"), py::arg("F"), py::arg("ratio"), py::arg("output_filename"))
                .def("get_vertices", &MeshSimplify::get_vertices)
                .def("get_faces", &MeshSimplify::get_faces)
                .def("process", &MeshSimplify::process)
            // 다른 필요한 메소드들을 여기에 추가하세요.
                ;
    }

} // namespace qslim
