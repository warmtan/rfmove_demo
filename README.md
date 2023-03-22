## https://github.com/mvig-robotflow/rfmove.git
## 此项目去除了ros 保留了moveit结合了pybullet作为显示页面
    去除ikfast 相关文件配置
    修改了extern 里面 franka_ikfast_plugin  kdl_kinematics_plugin
    在src文件目录下 python文件夹下面的cmakelist去除 ik相关内容
    目前只保留了kdl ik

## 以下为安装报错，此版本使用了ubuntu16.04 ros（kinetic）与moveit（kinetic）C++源码 
----------------------------------
    ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0
    conda install libffi=3.3 对比
----------------------------------
    console.h error
    sudo apt install libconsole-bridge-dev
----------------------------------
    https://blog.csdn.net/weixin_43837968/article/details/115257575