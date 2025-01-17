#ifndef STATE_INTERFACE_HPP
#define STATE_INTERFACE_HPP
#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
namespace hardware_interface
{
    typedef std::unordered_map<std::string, std::vector<double>> StateInterfaceDouble;
    typedef std::unordered_map<std::string, std::vector<int>> StateInterfaceInt;
    typedef std::unordered_map<std::string, std::vector<bool>> StateInterfaceBool;
    typedef std::tuple<StateInterfaceDouble, StateInterfaceInt, StateInterfaceBool> StateType;
    struct StateInterface
    {
        template <typename T>
        std::vector<T> & get(const std::string &name)
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(state_)[name];
        }
        template <typename T>
        const std::vector<T> & get (const std::string &name) const
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(state_).at(name);
        }

        template <typename T>
        std::unordered_map<std::string, std::vector<T>> & get()
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(state_);
        }
        template <typename T>
        const std::unordered_map<std::string, std::vector<T>> & get() const
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(state_);
        }

        StateType state_;
    };
} // namespace hardware

#endif // STATE_INTERFACE_HPP