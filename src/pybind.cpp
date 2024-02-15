#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "MeshSimplify.h" // 경로가 올바른지 확인하세요.

namespace py = pybind11;
namespace qslim {

    PYBIND11_MODULE(QSlim, m) {
        py::class_<MeshSimplify>(m, "MeshSimplify")
                .def(py::init<Eigen::MatrixXd &, Eigen::MatrixXi &>(),
                     py::arg("V"), py::arg("F"))
                .def("get_vertices", &MeshSimplify::get_vertices)
                .def("get_faces", &MeshSimplify::get_faces)
                .def("process", [](MeshSimplify &self, double ratio) { // ratio 인자 추가
                    self.process(ratio); // process 메소드를 호출하면서 ratio를 전달
                    return py::make_tuple(self.get_vertices(), self.get_faces()); // 수정된 V와 F를 튜플로 묶어 반환
                }, py::arg("ratio")) // ratio 인자 명시
                ;
    }


} // namespace qslim
