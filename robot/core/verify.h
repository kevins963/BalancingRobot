#ifndef ROBOT_CORE_VERIFY_H_
#define ROBOT_CORE_VERIFY_H_

void Verify_Handler(char* loc, char* line);

#define VERIFY(test)                    \
  if (!test) {                          \
    Verify_Handler(__func__, __line__); \
  }

#define DVERIFY(test) \
#ifdef _DEBUG do {} \
  while (0)           \
    ;                 \
                      \
#else VERIFY(test) #endif

#endif  // ROBOT_CORE_VERIFY_H_
