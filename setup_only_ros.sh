if [ ! -d "external/rpclib/rpclib-2.3.0" ]; then
    echo "*********************************************************************************************"
    echo "Downloading rpclib..."
    echo "*********************************************************************************************"

    wget https://github.com/rpclib/rpclib/archive/v2.3.0.zip

    # remove previous versions
    rm -rf "external/rpclib"

    mkdir -p "external/rpclib"
    unzip -q v2.3.0.zip -d external/rpclib
    rm v2.3.0.zip
fi

echo "Installing Eigen library..."

if [ ! -d "AirLib/deps/eigen3" ]; then
    echo "Downloading Eigen..."
    wget -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
    unzip -q eigen3.zip -d temp_eigen
    mkdir -p AirLib/deps/eigen3
    mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3
    rm -rf temp_eigen
    rm eigen3.zip
else
    echo "Eigen is already installed."
fi
