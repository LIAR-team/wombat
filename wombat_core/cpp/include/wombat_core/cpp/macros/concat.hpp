// Copyright 2024 Soragna Alberto.

#pragma once

/*
 * Concatenate string literals A and B.
 * Usage:
 * #define STR1 "hello+"
 * std::cout << WOMBAT_CORE_CONCAT_STR_LITERALS(STR1, "world") << std::endl;
 * Produces: hello+world
 * This will cause a compile-time error if used with variables
 */
#define WOMBAT_CORE_CONCAT_STR_LITERALS(A, B) A B
