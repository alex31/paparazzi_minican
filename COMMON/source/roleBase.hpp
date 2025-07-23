#pragma once

#include <ch.h>
#include <hal.h>
#include "roleStatus.hpp"

/**
 * @brief Base interface for all roles connected to a UAVCAN node.
 *
 * Any role class should inherit from RoleBase and implement the
 * subscribe() and start() methods for bus interaction.
 */
class RoleBase {
public:
  /**
   * @brief Subscribe the role to the required messages on the UAVCAN node.
   * @param node Reference to the UAVCAN node used for subscription.
   * @return Device status after subscription.
   */
  virtual DeviceStatus subscribe(UAVCAN::Node& node) = 0;

  /**
   * @brief Start the role and initialize its features.
   * @param node Reference to the UAVCAN node to use.
   * @return Device status after startup.
   */
  virtual DeviceStatus start(UAVCAN::Node& node) = 0;

  /**
   * @brief Virtual destructor for polymorphic deletion.
   */
  virtual ~RoleBase() = default;

protected:
  /**
   * @brief Pointer to the UAVCAN node associated with this role.
   */
  UAVCAN::Node *m_node = nullptr;
};

/**
 * @brief CRTP helper for singleton management in roles.
 *
 * Allows each derived class to be accessible via a unique static pointer (singleton).
 * This pattern simplifies using static trampolines to bind C callbacks to instance methods.
 *
 * @tparam Derived The class inheriting from RoleCrtp<Derived>.
 */
template<typename Derived>
class RoleCrtp {
public:
  /**
   * @brief Constructor. Ensures the singleton is unique for each derived class.
   */
  RoleCrtp() {
    chDbgAssert(singleton == nullptr, "Singleton not respected");
    singleton = static_cast<Derived *>(this);
  }

protected:
  /**
   * @brief Static singleton pointer for each derived class.
   */
  static inline Derived* singleton = nullptr;

  /**
   * @brief Declare Trampoline as friend to allow access to singleton.
   */
  template<auto>
  friend struct Trampoline;
};

/**
 * @brief Generic helper to convert a member function to a static trampoline.
 *
 * Allows using UAVCAN C-style callbacks by redirecting the call to the singleton instance.
 * Usage: Trampoline<&MyClass::myMethod>::fn
 *
 * @tparam Method Pointer to the member function to invoke.
 */
template<auto Method>
struct Trampoline;

/**
 * @brief Trampoline specialization for member functions.
 *
 * @tparam T     Class type.
 * @tparam Args  Argument types for the method.
 * @tparam Method Pointer to the member function of T.
 */
template<class T, typename... Args, void (T::*Method)(Args...)>
struct Trampoline<Method> {
    /**
     * @brief Static trampoline function compatible with C callbacks.
     *        Redirects the call to the singleton's member method.
     * @param args Arguments to forward to the member method.
     */
    static void fn(Args... args) {
        if (T::singleton)
            (T::singleton->*Method)(args...);
    }
};
