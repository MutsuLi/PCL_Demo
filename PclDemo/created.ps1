# 环境变量检查
$CHECK_LIST=@("PCL_ROOT", "OPENNI2_REDIST64", "OPENNI2_INCLUDE64", "OPENNI2_LIB64")
foreach($VARNAME in $CHECK_LIST){
  if ([String]::IsNullOrEmpty([System.Environment]::GetEnvironmentVariable($VARNAME))){
    $message = "未设置 " +$VARNAME +" 变量"
    echo $message
  }
}