/**
 * @file typeNameUtil.h
 * @brief Compile-time type name extraction utility for C++ templates.
 * @details Provides a header-only utility for obtaining human-readable names of C++ types
 *          at compile time using compiler introspection. Essential for telemetry and debugging
 *          of template-based classes.
 */

#pragma once 

/**
 * @brief Helper function to extract the type name from the __PRETTY_FUNCTION__ macro.
 * @details This constexpr function recursively scans through the pretty function string
 *          from the GCC/Clang __PRETTY_FUNCTION__ macro to find where the type name begins.
 *          The type name starts 2 characters after the '=' sign (after "= ").
 *
 *          **How it works:**
 *          1. The __PRETTY_FUNCTION__ macro expands to something like:
 *             `constexpr const char* get_typename() [T = float]`
 *          2. We scan for the '=' character which precedes the type name
 *          3. Once found, we return the pointer 2 positions after '=' (skipping "= ")
 *          4. The returned pointer points to the start of the type name (e.g., "float")
 *
 * @param s Pointer to the current character in the __PRETTY_FUNCTION__ string.
 *          Initially passed as __PRETTY_FUNCTION__, then recursively advanced.
 *
 * @return const char* A pointer to the first character of the type name within the string.
 *
 * @note **Compile-Time Execution:** This is a constexpr function that executes entirely at
 *       compile time, producing zero runtime overhead. The result is a compile-time constant.
 *
 * @note **Recursive Implementation:** Recursively advances through the string character by
 *       character until the '=' is found. For most types, this requires scanning ~100-200 characters.
 *       Recursion depth is reasonable but can become deep for very complex template types.
 *
 * @note **Current Character:** The function checks if the current character is '=' and returns
 *       if true. Otherwise, it recursively calls itself with the next character (s + 1).
 *
 * @note **Null Terminator Handling:** The function does not explicitly check for end-of-string,
 *       relying on __PRETTY_FUNCTION__ always containing the '=' delimiter.
 *
 * @warning **GCC/Clang Only:** This utility uses __PRETTY_FUNCTION__, which is specific to
 *          GCC and Clang. MSVC uses __FUNCSIG__, which has a different format. Porting to
 *          MSVC would require a separate implementation.
 *
 * @warning **Pointer Validity:** The returned pointer points into the __PRETTY_FUNCTION__ string,
 *          which exists for the entire compile time. The lifetime is sufficient for compile-time
 *          constant generation, but the pointer should not be stored for use at runtime
 *          (it may be optimized away or have other lifetime issues).
 *
 * @see get_typename() for typical usage
 *
 * @example
 * Given `__PRETTY_FUNCTION__ = "constexpr const char* get_typename() [T = float]"`
 * - Initial call: find_type_start("constexpr const char* get_typename() [T = float]")
 * - Recursively advances until finding '=' at position ~50
 * - Returns pointer 2 positions after: "float]"
 * - The caller then extracts just "float" by parsing until ']'
 */
constexpr const char* find_type_start(const char* s) {
    return (*s == '=') ? (s + 2) : find_type_start(s + 1);
}

/**
 * @brief Obtains a human-readable C++ type name at compile time.
 * @details A template function that leverages compiler introspection to extract the name
 *          of the template parameter T as a string literal. The type name is extracted from
 *          the __PRETTY_FUNCTION__ macro, which includes the template instantiation details.
 *
 *          **Purpose:**
 *          In the Motion Framework, sensors and actuators are template-based (BaseSensor<T>, BaseActuator<T>).
 *          This utility allows each instance to automatically determine and report its type name
 *          without relying on explicit string names or RTTI typeid() calls.
 *
 *          **Typical Usage in Framework:**
 *          ```cpp
 *          // In BaseActuator<T> constructor:
 *          BaseActuator(const char* name) 
 *              : IActuator(name, get_typename<T>(), sizeof(T)) { }
 *          
 *          // For a float-actuator, this automatically fills:
 *          // dataType = "float", typeSize = 4
 *          ```
 *
 * @tparam T The type whose name is to be extracted.
 *           Can be any C++ type: primitive (int, float), standard library (std::vector),
 *           or custom classes. Complex types produce longer names.
 *
 * @return constexpr const char* A compile-time string literal containing the type name.
 *
 *         **Examples:**
 *         - get_typename<float>() → "float"
 *         - get_typename<int32_t>() → "int"  (or "std::int32_t" depending on context)
 *         - get_typename<MyClass>() → "MyClass"
 *
 * @note **Compile-Time Execution:** This function executes entirely at compile time.
 *       The return value is a string literal embedded in the compiled binary.
 *       Zero runtime overhead: no function call, no string copying.
 *
 * @note **String Literal:** The returned pointer points to a string literal in the binary's
 *       read-only data section. The lifetime is the entire program execution.
 *       Safe to store in class members or pass to other functions.
 *
 * @note **No Runtime Cost:** Unlike std::typeid(T).name() which performs runtime lookups,
 *       this solution is purely compile-time. Suitable for embedded systems with limited resources.
 *
 * @note **Platform Dependent:** The exact format of type names depends on the compiler's
 *       __PRETTY_FUNCTION__ implementation. Different compilers may produce slightly different
 *       strings. However, for common types (int, float, etc.), the output is standardized.
 *
 * @note **Complex Types:** For complex template types, the type name can be very long:
 *       ```cpp
 *       get_typename<std::vector<std::pair<int, float>>>()
 *       // Produces something like:
 *       // "std::vector<std::pair<int, float>, std::allocator<std::pair<int, float> > >"
 *       ```
 *       This is usually acceptable for metadata but may be unwieldy in logs.
 *
 * @warning **GCC/Clang Specific:** This implementation uses __PRETTY_FUNCTION__ from GCC/Clang.
 *          It will not work with MSVC (which uses __FUNCSIG__ with different format) or
 *          other compilers without modifications. Porting required for cross-platform use.
 *
 * @warning **Not Binary Names:** The returned strings are human-readable names, not C++ mangled
 *          names. They're suitable for logging/UI but should not be used for runtime type
 *          identification (use RTTI typeid() for that).
 *
 * @warning **Edge Cases:** For very exotic types (nested templates, function pointers),
 *          the extracted name might include extra context. Always test with your types
 *          if relying on exact string format.
 *
 * @see find_type_start() for the implementation detail
 *
 * @example
 * Usage in the Motion Framework:
 * @code
 * auto motor = ESP32DCMotor::Create("Left Motor", config);
 * // Internally calls: get_typename<float>()
 * // Stores: "float" as the command type
 * 
 * auto encoder = ESP32Encoder::Create("Right Encoder", config);
 * // Internally calls: get_typename<int32_t>()
 * // Stores: "int32_t" as the reading type
 * 
 * // Later, when serializing for telemetry:
 * printf("Motor type: %s\n", motor->GetDataType()); // Prints: "float"
 * printf("Encoder type: %s\n", encoder->GetDataType()); // Prints: "int32_t"
 * @endcode
 * 
 */
template <typename T>
constexpr const char* get_typename() {
    return find_type_start(__PRETTY_FUNCTION__);
}