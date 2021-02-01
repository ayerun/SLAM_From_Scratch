# Rigid 2D transformation Library
A library for handling transformations in SE(2).

# Conceptual Questions
1. **What is the difference between a class and a struct in C++?** The default access mode and inheritance mode are public for structs and private for classes.
2. **Why is Vector2D a struct and Transform2D a class (refer to at least 2 specic C++ core guidelines in your answer)?** Transform2D is a class because it has non-public members, and Vector2D is a struct because it only has public members (rule C.8 in C++ core guidelines). Transform2D is a class because the transformation between frames is invariant, and Vector2D is a struct because the vector coordinates can vary independently (rule C.2 in C++ core guidelines).
3. **Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?** To avoid unintended conversions, single-arguement constructors should be declared explicit (rule C.46 in C++ core guidelines).
4. **We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):**
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.
   - Which of the methods would you implement and why?
    * I could create a public method that uses the public members and returns the normalized Vector2D. I would implement this method because it accomplishes the task and abides by the C++ core guidelines.
    * I could create private members x_norm and y_norm and use a constructor to initialize these members. According to the C++ core guidelines structs should not have private members.
    * I could create public members x_norm and y_norm and use a constructor to initialize these members. The normalized vector is an invariant, so according to the C++ core guidelines these members should be in a class not a struct.
5. **Why is Transform2D::inv() declared const while Transform2D::operator*=() is not (Refer to C++ core guidelines in your answer)?**
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer
By default, member functions should be const unless it changes the object's observable state. Transform2D::inv() does not change the object's observable state, but Transform2D::operator*=() changes the object's observable state.