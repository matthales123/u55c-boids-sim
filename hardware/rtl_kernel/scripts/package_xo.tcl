if { $argc != 6 } {
  puts "ERROR: usage: package_xo.tcl <kernel_name> <top_name> <xo_path> <kernel_xml> <ip_root_dir> <part>"
  exit 1
}

set kernel_name [lindex $argv 0]
set top_name    [lindex $argv 1]
set xo_path     [file normalize [lindex $argv 2]]
set kernel_xml  [file normalize [lindex $argv 3]]
set ip_root_dir [file normalize [lindex $argv 4]]
set part_name   [lindex $argv 5]

set script_dir      [file dirname [info script]]
set rtl_kernel_dir  [file normalize [file join $script_dir ".."]]
set hardware_dir    [file normalize [file join $rtl_kernel_dir ".."]]
set rtl_src_dir     [file join $hardware_dir "rtl" "src"]
set project_dir     [file normalize [file join $hardware_dir ".."]]

file mkdir [file dirname $xo_path]
file mkdir $ip_root_dir

set tmp_prj_dir [file join $ip_root_dir "pack_project"]
create_project -force "${kernel_name}_pack" $tmp_prj_dir -part $part_name

set src_files [list \
  [file join $rtl_kernel_dir "src" "boids_step_rtl_kernel.sv"] \
  [file join $rtl_src_dir "boids_step_soa_dual.sv"] \
  [file join $rtl_src_dir "boids_update_spatial_dual.sv"] \
  [file join $rtl_src_dir "boids_update_spatial.sv"] \
  [file join $rtl_src_dir "bound_check.sv"] \
  [file join $rtl_src_dir "lookuptable_div.sv"] \
  [file join $rtl_src_dir "signed_mult.sv"] \
  [file join $rtl_src_dir "speed_norm_approx.sv"] \
  [file join $rtl_src_dir "speed_update.sv"] \
]

foreach src_file $src_files {
  if {![file exists $src_file]} {
    puts "ERROR: missing source file: $src_file"
    exit 2
  }
}

add_files -norecurse $src_files
set_property top $top_name [current_fileset]
update_compile_order -fileset sources_1

ipx::package_project -root_dir $ip_root_dir -vendor user.org -library rtl -taxonomy /KernelIP -import_files -set_current true
set core [ipx::current_core]

set_property name $kernel_name $core
set_property display_name $kernel_name $core
set_property description "RTL boids kernel wrapper for U55C" $core
set_property sdx_kernel true $core
set_property sdx_kernel_type rtl $core
set_property core_revision 1 $core

ipx::infer_bus_interface ap_clk xilinx.com:signal:clock_rtl:1.0 $core
ipx::infer_bus_interface ap_rst_n xilinx.com:signal:reset_rtl:1.0 $core

set required_busifs [list m_axi_gmem0 m_axi_gmem1 m_axi_gmem2 s_axi_control]
foreach bus_if $required_busifs {
  set bus_obj [ipx::get_bus_interfaces $bus_if -of_objects $core]
  if {[llength $bus_obj] == 0} {
    puts "ERROR: bus interface '$bus_if' was not inferred. Check top-level port naming."
    exit 3
  }
  ipx::associate_bus_interfaces -busif $bus_if -clock ap_clk $core
}

set clk_if [ipx::get_bus_interfaces ap_clk -of_objects $core]
if {[llength $clk_if] != 0} {
  set assoc_param [ipx::add_bus_parameter ASSOCIATED_BUSIF $clk_if]
  set_property value "m_axi_gmem0:m_axi_gmem1:m_axi_gmem2:s_axi_control" $assoc_param
}

ipx::create_xgui_files $core
ipx::update_checksums $core
ipx::save_core $core
ipx::check_integrity -quiet $core

close_project

if {![file exists $kernel_xml]} {
  puts "ERROR: kernel.xml not found: $kernel_xml"
  exit 4
}

package_xo \
  -xo_path $xo_path \
  -kernel_name $kernel_name \
  -kernel_xml $kernel_xml \
  -ip_directory $ip_root_dir

puts "INFO: packaged xo at $xo_path"
