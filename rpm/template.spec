Name:           ros-indigo-concert-service-waypoint-navigation
Version:        0.1.9
Release:        0%{?dist}
Summary:        ROS concert_service_waypoint_navigation package

Group:          Development/Libraries
License:        BSD
URL:            http://ros.org/wiki/concert_service_waypoint_navigation
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-indigo-concert-service-link-graph
Requires:       ros-indigo-rocon-apps
BuildRequires:  ros-indigo-catkin

%description
Command robot to navigate around

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
* Mon Mar 23 2015 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.9-0
- Autogenerated by Bloom

* Fri Feb 27 2015 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.8-0
- Autogenerated by Bloom

* Mon Feb 09 2015 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.7-0
- Autogenerated by Bloom

* Mon Jan 05 2015 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.6-0
- Autogenerated by Bloom

* Fri Dec 05 2014 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.5-0
- Autogenerated by Bloom

* Fri Nov 21 2014 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.4-0
- Autogenerated by Bloom

* Fri Nov 21 2014 Jihoon Lee <jihoonlee.in@gmail.com> - 0.1.2-0
- Autogenerated by Bloom

