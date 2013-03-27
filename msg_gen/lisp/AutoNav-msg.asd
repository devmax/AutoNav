
(cl:in-package :asdf)

(defsystem "AutoNav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "predictInternal" :depends-on ("_package_predictInternal"))
    (:file "_package_predictInternal" :depends-on ("_package"))
    (:file "filter_state" :depends-on ("_package_filter_state"))
    (:file "_package_filter_state" :depends-on ("_package"))
    (:file "offsets" :depends-on ("_package_offsets"))
    (:file "_package_offsets" :depends-on ("_package"))
    (:file "control_commands" :depends-on ("_package_control_commands"))
    (:file "_package_control_commands" :depends-on ("_package"))
    (:file "obs_IMU_XYZ" :depends-on ("_package_obs_IMU_XYZ"))
    (:file "_package_obs_IMU_XYZ" :depends-on ("_package"))
    (:file "obs_IMU_RPY" :depends-on ("_package_obs_IMU_RPY"))
    (:file "_package_obs_IMU_RPY" :depends-on ("_package"))
    (:file "eulerpose" :depends-on ("_package_eulerpose"))
    (:file "_package_eulerpose" :depends-on ("_package"))
    (:file "obs_tag" :depends-on ("_package_obs_tag"))
    (:file "_package_obs_tag" :depends-on ("_package"))
    (:file "predictUpTo" :depends-on ("_package_predictUpTo"))
    (:file "_package_predictUpTo" :depends-on ("_package"))
  ))