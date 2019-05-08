# Install Nnabla Cuda Ext
cd nnabla-ext-cuda
pip install -U -r python/requirements.txt
mkdir build
cd build
../../cmake-3.14.3-Linux-x86_64/bin/cmake -DNNABLA_DIR=../../nnabla -DCPPLIB_LIBRARY=../../nnabla/build/lib/libnnabla.so -DBUILD_PYTHON_PACKAGE=OFF -DBUILD_CPP_UTILS=ON ..
make -j $(nproc)
sudo make install
cd ../../