#ifndef ROBOT_CORE_UTIL_H
#define ROBOT_CORE_UTIL_H

#define FORWARD_DECLARE_STRUCT(name) typedef struct name name;
#define DEFINE_EMPTY_STRUCT(struct_name) \
  static const struct struct_name kEmpty##struct_name;

#define FORWARD_AND_EMPTY_STRUCT(name) \
  FORWARD_DECLARE_STRUCT(name)         \
  DEFINE_EMPTY_STRUCT(name)

#define InitClass(class_name) *p_this = kEmpty##class_name;

#define InitDerivedClass(derived_class, base_class, ...) \
  InitClass(derived_class);                              \
  base_class##_Init(&p_this->base, ##__VA_ARGS__);

#endif  // ROBOT_CORE_UTIL_H
