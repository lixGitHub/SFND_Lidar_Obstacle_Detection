rm build/* -rf

cd build
cmake ..
make
cd -

./build/environment