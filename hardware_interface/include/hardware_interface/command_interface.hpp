#ifndef COMMAND_INTERFACE_HPP
#define COMMAND_INTERFACE_HPP
#include <map>
#include <unordered_map>
#include <string>
#include <vector>
#include <tuple>
namespace hardware_interface
{
    typedef std::unordered_map<std::string, std::vector<double>> CommandInterfaceDouble;
    typedef std::unordered_map<std::string, std::vector<int>> CommandInterfaceInt;
    typedef std::unordered_map<std::string, std::vector<bool>> CommandInterfaceBool;
    typedef std::tuple<CommandInterfaceDouble, CommandInterfaceInt, CommandInterfaceBool> CommandType;

    struct CommandInterface
    {
        template <typename T>
        std::vector<T> &get(const std::string &name)
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(command_)[name];
        }
        template <typename T>
        const std::vector<T> &get(const std::string &name) const
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(command_).at(name);
        }

        template <typename T>
        std::unordered_map<std::string, std::vector<T>> &get()
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(command_);
        }
        template <typename T>
        const std::unordered_map<std::string, std::vector<T>> &get() const
        {
            return std::get<std::unordered_map<std::string, std::vector<T>>>(command_);
        }
        void clear()
        {
            std::get<0>(command_).clear();
            std::get<1>(command_).clear();
            std::get<2>(command_).clear();
        }
        CommandType command_;
    };

} // namespace hardware

#endif // COMMAND_INTERFACE_HPP