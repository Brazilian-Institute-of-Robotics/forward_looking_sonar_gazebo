// Copyright 2018 Brazilian Intitute of Robotics"

#include <sdf/sdf.hh>
#include <string>

namespace gazebo
{
class SDFTool
{
public:
    template<typename T>
    static T GetSDFElement(const sdf::ElementPtr &_sdf, const std::string &_nameElement,
                            const std::string &_parent = "")
    {
        sdf::ElementPtr parentSdf = _sdf;
        if (!_parent.empty())
        {
            parentSdf = _sdf->GetElement(_parent);
            GZ_ASSERT(parentSdf != nullptr, (_parent + " is not set").c_str());
        }
        GZ_ASSERT(parentSdf->HasElement(_nameElement), (_nameElement + " is not set").c_str());
        return parentSdf->Get<T>(_nameElement);
    }
};
}  // namespace gazebo
