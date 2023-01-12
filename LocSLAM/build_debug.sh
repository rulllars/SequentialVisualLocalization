git submodule update --init --recursive

echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building LocSLAM ..."

mkdir -p build_debug
if [ -e build ]; then
    rm build
fi
ln -rs build_debug build
cd build
#cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j
