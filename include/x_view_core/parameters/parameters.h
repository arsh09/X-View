#ifndef X_VIEW_PARAMETERS_H
#define X_VIEW_PARAMETERS_H

#include <x_view_core/x_view_types.h>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>

namespace x_view {

class Parameters {
 public:
  Parameters(const std::string& name = "")
      : indentation(""),
        name_(name) {}

  /// \brief Checks if the Parameters instance contains the key passed as
  /// parameter.
  bool has(const std::string& name) const {
    return (properties_.find(name) != properties_.end());
  }

  /// \brief Set a boolean property.
  void setBoolean(const std::string& name, const bool& value);

  /**
   * \brief Get a boolean property.
   * \param name Key associated to the boolean value to retrieve.
   * \return Boolean associated to the key passed as parameter.
   */
  bool getBoolean(const std::string& name) const;

  /**
   * \brief Get a boolean property and, if the key is not registered return
   * the default value passed as argument.
   * \param name Key associated to the boolean value to retrieve.
   * \param defaultValue Boolean value returned in case the key is not found
   * in the properties.
   * \return Boolean associated to the key passed as parameter.
   */
  bool getBoolean(const std::string& name, const bool& defaultValue) const;

  /// \brief Set an integer property.
  void setInteger(const std::string& name, const int& value);

  /**
   * \brief Get a integer property.
   * \param name Key associated to the integer value to retrieve.
   * \return Integer associated to the key passed as parameter.
   */
  int getInteger(const std::string& name) const;

  /**
   * \brief Get a integer property and, if the key is not registered return
   * the default value passed as argument.
   * \param name Key associated to the integer value to retrieve.
   * \param defaultValue Integer value returned in case the key is not found
   * in the properties.
   * \return Integer associated to the key passed as parameter.
   */
  int getInteger(const std::string& name, const int& defaultValue) const;

  /// \brief Set a floating point value property.
  void setFloat(const std::string& name, const real_t& value);

  /**
   * \brief Get a floating point value property.
   * \param name Key associated to the floating point value to retrieve.
   * \return Floating point value associated to the key passed as parameter.
   */
  real_t getFloat(const std::string& name) const;

  /**
   * \brief Get a floating point value property and, if the key is not
   * registered return the default value passed as argument.
   * \param name Key associated to the floating point value to retrieve.
   * \param defaultValue floating point value returned in case the key is not
   * found in the properties.
   * \return Floating point value associated to the key passed as parameter.
   */
  real_t getFloat(const std::string& name, const real_t& defaultValue) const;

  /// \brief Set a string property.
  void setString(const std::string& name, const std::string& value);

  /**
   * \brief Get a string property.
   * \param name Key associated to the string value to retrieve.
   * \return String associated to the key passed as parameter.
   */
  std::string getString(const std::string& name) const;

  /**
   * \brief Get a string property and, if the key is not registered return
   * the default value passed as argument.
   * \param name Key associated to the string value to retrieve.
   * \param defaultValue String value returned in case the key is not found
   * in the properties.
   * \return String associated to the key passed as parameter.
   */
  std::string getString(const std::string& name,
                        const std::string& defaultValue) const;

  /// \brief Set a Parameter object as child.
  /// \note The calling Parameter instance acquires ownership of the children
  /// pointed by the unique pointer passed as argument.
  void addChildPropertyList(const std::string& name,
                            std::unique_ptr<Parameters> value);

  /**
   * \brief Get a const reference to the pointer pointing to the child property
   * associated with the key passed as parameter.
   * \param name Key associated to the child property to retrieve.
   * \return Child property associated to the key passed as parameter.
   */
  const std::unique_ptr<Parameters>& getChildPropertyList(const std::string& name) const;

  /**
   * \brief Get a reference to the pointer pointing to the child property
   * associated with the key passed as parameter.
   * \param name Key associated to the child property to retrieve.
   * \return Child property associated to the key passed as parameter.
   */
  std::unique_ptr<Parameters>& getChildPropertyList(const std::string& name);

  /// \brief Writes all parameters as a string.
  std::string toString() const;

  /// \brief Returns the name of the Parameters instance.
  const std::string& name() const {
    return name_;
  }

  /// \brief Indentation to be used for current Parameters instance when
  /// being written to a string.
  std::string indentation;

 private:
  /// \brief  Custom variant data type (stores one of boolean/integer/float/..)
  struct Property {
    /// \brief Tag storing the type contained in the current property.
    enum {
      Boolean_type, Integer_type, Float_type,
      String_type, PropertyList_type
    } type;

    /// \brief Only one member at the time of the struct is used depending on
    /// the Property type.
    struct Value {
      Value() : Boolean_value(false) {}
      ~Value() {}

      bool Boolean_value;
      int Integer_value;
      real_t Float_value;
      std::string String_value;
      std::unique_ptr<Parameters> PropertyList_value;
    } value;

    Property() : type(Boolean_type) {}

    std::string toString() const;
  };

  /// \brief Container of properties keyed by strings.
  std::unordered_map<std::string, Property> properties_;
  std::list<std::string> insertion_order_;
  const std::string name_;

};

}

#endif //X_VIEW_PARAMETERS_H
