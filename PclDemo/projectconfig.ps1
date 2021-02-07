$PCL_3RDPARTY_BOOST = ($env:PCL_ROOT + "/3rdParty/Boost")
$PCL_3RDPARTY_EIGEN = ($env:PCL_ROOT + "/3rdParty/EIGEN")
$PCL_3RDPARTY_VTK = ($env:PCL_ROOT + "/3rdParty/VTK")
$PCL_3RDPARTY_QHULL = ($env:PCL_ROOT + "/3rdParty/QHULL")
$PCL_3RDPARTY_FLANN = ($env:PCL_ROOT + "/3rdParty/FLANN")

# 查找 OPENNI2 相关变量
$OPENNI2_ROOT = (Get-Item $env:OPENNI2_INCLUDE64).Parent.FullName
$OPENNI2_TOOLS = (Get-ChildItem -Directory -Filter "Tools" $OPENNI2_ROOT)[0].FullName

# 查找PCL相关变量
$PCL_ROOT_INCLUDE = (Get-ChildItem -Directory -Filter "pcl-*" ($env:PCL_ROOT + "/include"))[0].FullName
$PCL_BOOST_INCLUDE = (Get-ChildItem -Directory -Depth 2 -Filter "boost-*" $PCL_3RDPARTY_BOOST | ? {$_.Parent.BaseName -eq 'include' })[0].FullName
$PCL_EIGEN_INCLUDE = (Get-ChildItem -Directory -Filter "eigen*" $PCL_3RDPARTY_EIGEN)[0].FullName
$PCL_VTK_INCLUDE = (Get-ChildItem -Directory -Depth 2 -Filter "vtk-*" $PCL_3RDPARTY_VTK | ? {$_.Parent.BaseName -eq 'include' })[0].FullName
$PCL_QHULL_INCLUDE = (Get-ChildItem -Directory -Filter "include" $PCL_3RDPARTY_QHULL)[0].FullName
$PCL_FLANN_INCLUDE = (Get-ChildItem -Directory -Filter "include" $PCL_3RDPARTY_FLANN)[0].FullName

$PCL_ROOT_LIB = (Get-ChildItem -Directory -Filter "lib" $env:PCL_ROOT)[0].FullName
$PCL_BOOST_LIB = (Get-ChildItem -Directory -Filter "lib" $PCL_3RDPARTY_BOOST)[0].FullName
$PCL_VTK_LIB = (Get-ChildItem -Directory -Filter "lib" $PCL_3RDPARTY_VTK)[0].FullName
$PCL_QHULL_LIB = (Get-ChildItem -Directory -Filter "lib" $PCL_3RDPARTY_QHULL)[0].FullName
$PCL_FLANN_LIB = (Get-ChildItem -Directory -Filter "lib" $PCL_3RDPARTY_FLANN)[0].FullName

$VARLIST = @("OPENNI2_TOOLS", "PCL_BOOST_INCLUDE", "PCL_BOOST_LIB", "PCL_EIGEN_INCLUDE", "PCL_FLANN_INCLUDE", "PCL_FLANN_LIB", "PCL_QHULL_INCLUDE", "PCL_QHULL_LIB", "PCL_ROOT_INCLUDE", "PCL_ROOT_LIB", "PCL_VTK_INCLUDE", "PCL_VTK_LIB")

Get-Variable -Include $VARLIST