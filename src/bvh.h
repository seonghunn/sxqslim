//
// Created by seonghun on 8/21/23.
//

#ifndef QEM_BVH_H
#define QEM_BVH_H

#include <igl/AABB.h>

namespace qslim{
    template <typename DerivedV, int DIM>
    class BVH : public igl::AABB<DerivedV, DIM>
    {
    public:
        BVH() : igl::AABB<DerivedV, DIM>(){}

        void update()
        {
            // 여기에 update 메소드 구현
        }
    };

}


#endif //QEM_BVH_H
