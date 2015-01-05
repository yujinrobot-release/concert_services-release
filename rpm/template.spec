Name:           ros-indigo-concert-service-gazebo
Version:        0.1.6
Release:        0%{?dist}
Summary:        ROS concert_service_gazebo package

Group:          Development/Libraries
License:        BSD
URL:            http://ros.org/wiki/concert_service_gazebo
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-indigo-gateway-msgs
Requires:       ros-indigo-kobuki-gazebo
Requires:       ros-indigo-rocon-console
Requires:       ros-indigo-rocon-gateway-utils
Requires:       ros-indigo-rocon-launch
Requires:       ros-indigo-rocon-python-utils
Requires:       ros-indigo-rocon-std-msgs
Requires:       ros-indigo-roslib
Requires:       ros-indigo-rospy
Requires:       ros-indigo-turtlebot-gazebo
BuildRequires:  ros-indigo-catkin

%description
Sets up the gazebo robot manager as a service to assist in spawning/killing
robots as concert clients.

%prep
%setup -q

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
mkdir -p obj-%{_target_platform} && cd obj-%{_target_platform}
%cmake .. \
        -UINCLUDE_INSTALL_DIR \
        -ULIB_INSTALL_DIR \
        -USYSCONF_INSTALL_DIR \
        -USHARE_INSTALL_PREFIX \
        -ULIB_SUFFIX \
        -DCMAKE_INSTALL_PREFIX="/opt/ros/indigo" \
        -DCMAKE_PREFIX_PATH="/opt/ros/indigo" \
        -DSETUPTOOLS_DEB_LAYOUT=OFF \
        -DCATKIN_BUILD_BINARY_PACKAGE="1" \

make %{?_smp_mflags}

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree that was dropped by catkin, and source it.  It will
# set things like CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/indigo/setup.sh" ]; then . "/opt/ros/indigo/setup.sh"; fi
cd obj-%{_target_platform}
make %{?_smp_mflags} install DESTDIR=%{buildroot}

%files
/opt/ros/indigo

%changelog
* Mon Jan 05 2015 Daniel Stonier <d.stonier@gmail.com> - 0.1.6-0
- Autogenerated by Bloom

