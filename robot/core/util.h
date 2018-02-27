#ifndef ROBOT_CORE_UTIL_H
#define ROBOT_CORE_UTIL_H

#define FORWARD_DECLARE_STRUCT(name) typedef struct name name;
#define DEFINE_EMPTY_STRUCT(struct_name) \
  static const struct struct_name kEmpty##struct_name;

#define BEGIN_STRUCT(name)     \
  FORWARD_DECLARE_STRUCT(name) \
  struct name {
#define END_STRUCT(name) \
  }                      \
  ;                      \
  DEFINE_EMPTY_STRUCT(name)

#define InitClass(class_name) *p_this = kEmpty##class_name;

#define InitDerivedClass(derived_class, base_class, ...) \
  InitClass(derived_class);                              \
  base_class##_Init(&p_this->base, ##__VA_ARGS__);

#define BIT_POS_0 0x00
#define BIT_POS_1 0x01
#define BIT_POS_2 0x02
#define BIT_POS_3 0x03
#define BIT_POS_4 0x04
#define BIT_POS_5 0x05
#define BIT_POS_6 0x06
#define BIT_POS_7 0x07

#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

#define BIT_MASK_SIZE_1 (0x01)
#define BIT_MASK_SIZE_2 (0x03)
#define BIT_MASK_SIZE_3 (0x07)
#define BIT_MASK_SIZE_4 (0x0F)
#define BIT_MASK_SIZE_5 (0x1F)
#define BIT_MASK_SIZE_6 (0x3F)
#define BIT_MASK_SIZE_7 (0x7F)
#define BIT_MASK_SIZE_8 (0xFF)

#define MASKED_VALUE(val, bit_pos, mask_size) \
  (((val) & (mask_size)) << (bit_pos))

#endif  // ROBOT_CORE_UTIL_H
