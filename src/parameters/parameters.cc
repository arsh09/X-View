#include <x_view_core/parameters/parameters.h>

#include <glog/logging.h>

namespace x_view {

#define DEFINE_SETTER_ACCESSOR(Type, TypeName) \
    void Parameters::set##TypeName(const std::string &name, const Type &value) { \
        if (properties_.find(name) != properties_.end()) \
            LOG(WARNING) << "Property " << name \
                         << " was specified multiple times!"; \
        auto &prop = properties_[name]; \
        insertion_order_.push_back(name); \
        prop.value.TypeName##_value = value; \
        prop.type = Property::TypeName##_type; \
    }

#define DEFINE_GETTER_ACCESSOR(Type, TypeName) \
    Type Parameters::get##TypeName(const std::string &name) const { \
        auto it = properties_.find(name); \
        if (it == properties_.end()) \
            LOG(ERROR)<< "Property "<< name << " is missing!"; \
        if (it->second.type != Property::TypeName##_type) \
            LOG(ERROR) << "Property " << name << " has the wrong type!"; \
        return it->second.value.TypeName##_value; \
    }

#define DEFINE_DEFAULT_GETTER_ACCESSOR(Type, TypeName) \
    Type Parameters::get##TypeName(const std::string &name, const Type &defVal) const { \
        auto it = properties_.find(name); \
        if (it == properties_.end()) \
            return defVal; \
        if (it->second.type != Property::TypeName##_type) \
            LOG(ERROR) << "Property " << name  << " has the wrong type!"; \
        return it->second.value.TypeName##_value; \
    }

#define DEFINE_PROPERTY_ACCESSOR(Type, TypeName) \
    DEFINE_SETTER_ACCESSOR(Type, TypeName) \
    DEFINE_GETTER_ACCESSOR(Type, TypeName) \
    DEFINE_DEFAULT_GETTER_ACCESSOR(Type, TypeName)

DEFINE_PROPERTY_ACCESSOR(bool, Boolean)
DEFINE_PROPERTY_ACCESSOR(int, Integer)
DEFINE_PROPERTY_ACCESSOR(real_t, Float)
DEFINE_PROPERTY_ACCESSOR(std::string, String)

void Parameters::addChildPropertyList(const std::string& name,
                                      std::unique_ptr<Parameters> value) {
  if (properties_.find(name) != properties_.end())
    LOG(WARNING) << "Property " << name
                 << " was specified multiple times!";
  auto& prop = properties_[name];
  insertion_order_.push_back(name);
  value->indentation = indentation + "     ";
  prop.value.PropertyList_value = std::move(value);
  prop.type = Property::PropertyList_type;

}

const std::unique_ptr<Parameters>& Parameters::getChildPropertyList(const std::string& name) const {
  auto it = properties_.find(name);
  if (it == properties_.end())
    LOG(ERROR) << "Property " << name << " is missing!";
  if (it->second.type != Property::PropertyList_type)
    LOG(ERROR) << "Property " << name << " has the wrong type!";
  return it->second.value.PropertyList_value;
}

std::unique_ptr<Parameters>& Parameters::getChildPropertyList(const std::string& name) {
  auto it = properties_.find(name);
  if (it == properties_.end())
    LOG(ERROR) << "Property " << name << " is missing!";
  if (it->second.type != Property::PropertyList_type)
    LOG(ERROR) << "Property " << name << " has the wrong type!";
  return it->second.value.PropertyList_value;
}

std::string Parameters::toString() const {
  std::string s = "\n";
  for(const std::string& key : insertion_order_) {
    const auto iter = properties_.find(key);
    const  Property& property = iter->second;
    if(property.type == Property::PropertyList_type) {
      s += indentation + "|\n";
      s += indentation + "+->";
      s += property.value.PropertyList_value->name() + ":";
    } else {
      s += indentation + key + ": ";
    }
    s += property.toString() + "\n";
  }
  return s;
}

std::string Parameters::Property::toString() const {
  switch(type) {
    case Property::Boolean_type:
      return value.Boolean_value ? "True" : "False";
    case Property::Integer_type:
      return std::to_string(value.Integer_value);
    case Property::Float_type:
      return  std::to_string(value.Float_value);
    case Property::String_type:
      return value.String_value;
    case Property::PropertyList_type:
      return value.PropertyList_value->toString();
    default:
      return "UNKNOWN PARAMETER TYPE";
    }
}

}

