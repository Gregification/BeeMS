/*
 * is_base_of_custom.hpp
 *
 *  Created on: May 11, 2025
 *      Author: turtl
 */
/*
 * the ti compiler is missing STL, this is a alternative
 */

#ifndef UTILS_CUSTOM_IS_BASE_OF_CUSTOM_HPP_
#define UTILS_CUSTOM_IS_BASE_OF_CUSTOM_HPP_

template <typename Derived, typename Base>
class is_base_of_custom {
private:
    static char test(Base*);
    static int test(...);

public:
    static const bool value = sizeof(test(static_cast<Derived*>(nullptr))) == sizeof(char);
};

#endif /* UTILS_CUSTOM_IS_BASE_OF_CUSTOM_HPP_ */
