import geonimetry
import rclnim/rosinterfaceimporters

importInterface geometry_msgs/msg/[point, vector3, quaternion, transform], transform.Transform as TrnasformMsg

proc to*(value: SomeNumber, T: typedesc[SomeNumber]): auto =
  T(value)

proc to*[T](value: T, _: typedesc[string]): string =
  $value

proc to*[T](value: Vector3, _: typedesc[Vector[3, T]]): Vector[3, T] =
  initVector([value.x.to(T), value.y.to(T), value.z.to(T)])

proc to*[T](value: Point, _: typedesc[Vector[3, T]]): Vector[3, T] =
  initVector([value.x.to(T), value.y.to(T), value.z.to(T)])

proc to*[T](value: Quaternion, _: typedesc[Quat[T]]): Quat[T] =
  quat(value.x.to(T), value.y.to(T), value.z.to(T), value.w.to(T))

proc to*[T](value: TrnasformMsg, _: typedesc[Transform[3, T]]): Transform[3, T] =
  transform3(value.translation.to(Vector[3, T]), value.rotation.to(Quat[T]))
