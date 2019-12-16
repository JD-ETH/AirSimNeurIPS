# code generator build script

set -e

cd ../../ACADOtoolkit/build
make
cd ../../ACADOtoolkit/examples/code_generation/mpc_mhe/
./code_generation_quadrotor_model_thrustrates
cd ~/competition/KiteRunner/mpc_control
rm -rf quadrotor_mpc_codegen
mv ../../ACADOtoolkit/examples/code_generation/mpc_mhe/quadrotor_mpc_codegen ./
cp -r ../../rpg_mpc/externals/qpoases quadrotor_mpc_codegen/
cp python/* quadrotor_mpc_codegen/
cd quadrotor_mpc_codegen
python3 setup.py build install --force
cd ..
