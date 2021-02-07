
# Step1 检查 PCL_ROOT, OPENNI2_REDIST64, OPENNI2_INCLUDE64, OPENNI2_LIB64 是否已设置
$CHECK_LIST=@("PCL_ROOT", "OPENNI2_REDIST64", "OPENNI2_INCLUDE64", "OPENNI2_LIB64")
foreach($VARNAME in $CHECK_LIST){
  if ([String]::IsNullOrEmpty([System.Environment]::GetEnvironmentVariable($VARNAME))){
    $message = "no setting"  +$VARNAME +"param,stop program"
    echo $message
    exit
  }
}

# Step2: 基础路径设置
$OPENNI2_ROOT = (Get-Item $env:OPENNI2_INCLUDE64).Parent.FullName
$OPENNI2_TOOLS = (Get-ChildItem -Directory -Filter "Tools" $OPENNI2_ROOT)[0].FullName
$PCL_3RDPARTY_BOOST = ($env:PCL_ROOT + "\3rdParty\Boost")
$PCL_3RDPARTY_EIGEN = ($env:PCL_ROOT + "\3rdParty\EIGEN")
$PCL_3RDPARTY_FLANN = ($env:PCL_ROOT + "\3rdParty\FLANN")
$PCL_3RDPARTY_QHULL = ($env:PCL_ROOT + "\3rdParty\QHULL")
$PCL_3RDPARTY_VTK = ($env:PCL_ROOT + "\3rdParty\VTK")

# Step3 配置 PATH 环境变量

$PathList = $env:Path.Split(";");
$PathList_User = [System.Environment]::GetEnvironmentVariable("Path","User").Split(";");

$PCL_3RDPARTY_FLANN_bin = $PCL_3RDPARTY_FLANN + "\bin";
if( ! $PathList.Contains($PCL_3RDPARTY_FLANN_bin)){ $PathList_User += $PCL_3RDPARTY_FLANN_bin; }

$PCL_3RDPARTY_QHULL_bin = $PCL_3RDPARTY_QHULL + "\bin";
if( ! $PathList.Contains($PCL_3RDPARTY_QHULL_bin)){ $PathList_User += $PCL_3RDPARTY_QHULL_bin; }

$PCL_3RDPARTY_VTK_bin = $PCL_3RDPARTY_VTK + "\bin";
if( ! $PathList.Contains($PCL_3RDPARTY_VTK_bin)){ $PathList_User += $PCL_3RDPARTY_VTK_bin; }

if( ! $PathList.Contains($env:OPENNI2_REDIST64)){ $PathList_User += $env:OPENNI2_REDIST64; }
if( ! $PathList.Contains($OPENNI2_TOOLS)){ $PathList_User += $OPENNI2_TOOLS; }

[System.Environment]::SetEnvironmentVariable("Path", [String]::Join(";",$PathList_User),"User")
