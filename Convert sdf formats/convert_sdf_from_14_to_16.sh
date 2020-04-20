# usage:
# convert_sdf_from_14_to_16.sh map.world newmap.world
# magnetic_field kan måske bevares

xmlstarlet ed \
  -d '//friction' \
  -d '//visual/meta' \
  -d '//link/enable_wind' \
  -d '//magnetic_field' \
  -d '//gravity' \
  -d '//atmosphere' \
  -d '//audio' \
  -d '//state/iterations' \
  -d '//state/light' \
  -d '//scale' \
  -d '//gui/camera/projection_type' \
  -d '//world/wind' \
  -d '//axis/use_parent_model_frame' \
  -d '//axis/dynamics/spring_reference' \
  -d '//axis/dynamics/spring_stiffness' \
  -d '//sensor/imu' \
  -d '//visual/material/lighting' \
  -d '//camera/view_controller' \
  -d '//pose/@frame' \
  -d '//collide_bitmask' $1 | sed -e 's|sdf\ version="1.6"|sdf\ version="1.4"|'   > $2
 # --update "//sdf[@version='1.6']" --value "1.4" \
 #$1 > $2

