# satellite_orbit 模块源文件清单
#
# OPIR 卫星轨道运动模型：由轨道六根数与历元推进卫星 ECEF 位置/速度。
oneq_add_component(satellite_orbit_core SOURCES
    OrbitPropagator.cpp)
