//#line 2 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"
// *********************************************************
// 
// File autogenerated for the trajectory_generator package 
// by the dynamic_reconfigure package.
// Please do not edit.
// 
// ********************************************************/

/***********************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 ***********************************************************/

// Author: Blaise Gassend


#ifndef __trajectory_generator__TRAJECTORYGENERATORUICONFIG_H__
#define __trajectory_generator__TRAJECTORYGENERATORUICONFIG_H__

#include <dynamic_reconfigure/config_tools.h>
#include <limits>
#include <ros/node_handle.h>
#include <dynamic_reconfigure/ConfigDescription.h>
#include <dynamic_reconfigure/ParamDescription.h>
#include <dynamic_reconfigure/Group.h>
#include <dynamic_reconfigure/config_init_mutex.h>
#include <boost/any.hpp>

namespace trajectory_generator
{
  class TrajectoryGeneratorUIConfigStatics;
  
  class TrajectoryGeneratorUIConfig
  {
  public:
    class AbstractParamDescription : public dynamic_reconfigure::ParamDescription
    {
    public:
      AbstractParamDescription(std::string n, std::string t, uint32_t l, 
          std::string d, std::string e)
      {
        name = n;
        type = t;
        level = l;
        description = d;
        edit_method = e;
      }
      
      virtual void clamp(TrajectoryGeneratorUIConfig &config, const TrajectoryGeneratorUIConfig &max, const TrajectoryGeneratorUIConfig &min) const = 0;
      virtual void calcLevel(uint32_t &level, const TrajectoryGeneratorUIConfig &config1, const TrajectoryGeneratorUIConfig &config2) const = 0;
      virtual void fromServer(const ros::NodeHandle &nh, TrajectoryGeneratorUIConfig &config) const = 0;
      virtual void toServer(const ros::NodeHandle &nh, const TrajectoryGeneratorUIConfig &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, TrajectoryGeneratorUIConfig &config) const = 0;
      virtual void toMessage(dynamic_reconfigure::Config &msg, const TrajectoryGeneratorUIConfig &config) const = 0;
      virtual void getValue(const TrajectoryGeneratorUIConfig &config, boost::any &val) const = 0;
    };

    typedef boost::shared_ptr<AbstractParamDescription> AbstractParamDescriptionPtr;
    typedef boost::shared_ptr<const AbstractParamDescription> AbstractParamDescriptionConstPtr;
    
    template <class T>
    class ParamDescription : public AbstractParamDescription
    {
    public:
      ParamDescription(std::string name, std::string type, uint32_t level, 
          std::string description, std::string edit_method, T TrajectoryGeneratorUIConfig::* f) :
        AbstractParamDescription(name, type, level, description, edit_method),
        field(f)
      {}

      T (TrajectoryGeneratorUIConfig::* field);

      virtual void clamp(TrajectoryGeneratorUIConfig &config, const TrajectoryGeneratorUIConfig &max, const TrajectoryGeneratorUIConfig &min) const
      {
        if (config.*field > max.*field)
          config.*field = max.*field;
        
        if (config.*field < min.*field)
          config.*field = min.*field;
      }

      virtual void calcLevel(uint32_t &comb_level, const TrajectoryGeneratorUIConfig &config1, const TrajectoryGeneratorUIConfig &config2) const
      {
        if (config1.*field != config2.*field)
          comb_level |= level;
      }

      virtual void fromServer(const ros::NodeHandle &nh, TrajectoryGeneratorUIConfig &config) const
      {
        nh.getParam(name, config.*field);
      }

      virtual void toServer(const ros::NodeHandle &nh, const TrajectoryGeneratorUIConfig &config) const
      {
        nh.setParam(name, config.*field);
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, TrajectoryGeneratorUIConfig &config) const
      {
        return dynamic_reconfigure::ConfigTools::getParameter(msg, name, config.*field);
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const TrajectoryGeneratorUIConfig &config) const
      {
        dynamic_reconfigure::ConfigTools::appendParameter(msg, name, config.*field);
      }

      virtual void getValue(const TrajectoryGeneratorUIConfig &config, boost::any &val) const
      {
        val = config.*field;
      }
    };

    class AbstractGroupDescription : public dynamic_reconfigure::Group
    {
      public:
      AbstractGroupDescription(std::string n, std::string t, int p, int i, bool s)
      {
        name = n;
        type = t;
        parent = p;
        state = s;
        id = i;
      }

      std::vector<AbstractParamDescriptionConstPtr> abstract_parameters;
      bool state;

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &config) const = 0;
      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &config) const =0;
      virtual void updateParams(boost::any &cfg, TrajectoryGeneratorUIConfig &top) const= 0;
      virtual void setInitialState(boost::any &cfg) const = 0;


      void convertParams()
      {
        for(std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = abstract_parameters.begin(); i != abstract_parameters.end(); ++i)
        {
          parameters.push_back(dynamic_reconfigure::ParamDescription(**i));
        }
      }
    };

    typedef boost::shared_ptr<AbstractGroupDescription> AbstractGroupDescriptionPtr;
    typedef boost::shared_ptr<const AbstractGroupDescription> AbstractGroupDescriptionConstPtr;

    template<class T, class PT>
    class GroupDescription : public AbstractGroupDescription
    {
    public:
      GroupDescription(std::string name, std::string type, int parent, int id, bool s, T PT::* f) : AbstractGroupDescription(name, type, parent, id, s), field(f)
      {
      }

      GroupDescription(const GroupDescription<T, PT>& g): AbstractGroupDescription(g.name, g.type, g.parent, g.id, g.state), field(g.field), groups(g.groups)
      {
        parameters = g.parameters;
        abstract_parameters = g.abstract_parameters;
      }

      virtual bool fromMessage(const dynamic_reconfigure::Config &msg, boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        if(!dynamic_reconfigure::ConfigTools::getGroupState(msg, name, (*config).*field))
          return false;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          if(!(*i)->fromMessage(msg, n))
            return false;
        }

        return true;
      }

      virtual void setInitialState(boost::any &cfg) const
      {
        PT* config = boost::any_cast<PT*>(cfg);
        T* group = &((*config).*field);
        group->state = state;

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = boost::any(&((*config).*field));
          (*i)->setInitialState(n);
        }

      }

      virtual void updateParams(boost::any &cfg, TrajectoryGeneratorUIConfig &top) const
      {
        PT* config = boost::any_cast<PT*>(cfg);

        T* f = &((*config).*field);
        f->setParams(top, abstract_parameters);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          boost::any n = &((*config).*field);
          (*i)->updateParams(n, top);
        }
      }

      virtual void toMessage(dynamic_reconfigure::Config &msg, const boost::any &cfg) const
      {
        const PT config = boost::any_cast<PT>(cfg);
        dynamic_reconfigure::ConfigTools::appendGroup<T>(msg, name, id, parent, config.*field);

        for(std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = groups.begin(); i != groups.end(); ++i)
        {
          (*i)->toMessage(msg, config.*field);
        }
      }

      T (PT::* field);
      std::vector<TrajectoryGeneratorUIConfig::AbstractGroupDescriptionConstPtr> groups;
    };
    
class DEFAULT
{
  public:
    DEFAULT()
    {
      state = true;
      name = "Default";
    }

    void setParams(TrajectoryGeneratorUIConfig &config, const std::vector<AbstractParamDescriptionConstPtr> params)
    {
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator _i = params.begin(); _i != params.end(); ++_i)
      {
        boost::any val;
        (*_i)->getValue(config, val);

        if("vel_trajectory"==(*_i)->name){vel_trajectory = boost::any_cast<double>(val);}
        if("acc_trajectory"==(*_i)->name){acc_trajectory = boost::any_cast<double>(val);}
        if("vel_kinematics"==(*_i)->name){vel_kinematics = boost::any_cast<double>(val);}
        if("place_holder"==(*_i)->name){place_holder = boost::any_cast<bool>(val);}
      }
    }

    double vel_trajectory;
double acc_trajectory;
double vel_kinematics;
bool place_holder;

    bool state;
    std::string name;

    
}groups;



//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double vel_trajectory;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double acc_trajectory;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      double vel_kinematics;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      bool place_holder;
//#line 255 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

    bool __fromMessage__(dynamic_reconfigure::Config &msg)
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();

      int count = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        if ((*i)->fromMessage(msg, *this))
          count++;

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i ++)
      {
        if ((*i)->id == 0)
        {
          boost::any n = boost::any(this);
          (*i)->updateParams(n, *this);
          (*i)->fromMessage(msg, n);
        }
      }

      if (count != dynamic_reconfigure::ConfigTools::size(msg))
      {
        ROS_ERROR("TrajectoryGeneratorUIConfig::__fromMessage__ called with an unexpected parameter.");
        ROS_ERROR("Booleans:");
        for (unsigned int i = 0; i < msg.bools.size(); i++)
          ROS_ERROR("  %s", msg.bools[i].name.c_str());
        ROS_ERROR("Integers:");
        for (unsigned int i = 0; i < msg.ints.size(); i++)
          ROS_ERROR("  %s", msg.ints[i].name.c_str());
        ROS_ERROR("Doubles:");
        for (unsigned int i = 0; i < msg.doubles.size(); i++)
          ROS_ERROR("  %s", msg.doubles[i].name.c_str());
        ROS_ERROR("Strings:");
        for (unsigned int i = 0; i < msg.strs.size(); i++)
          ROS_ERROR("  %s", msg.strs[i].name.c_str());
        // @todo Check that there are no duplicates. Make this error more
        // explicit.
        return false;
      }
      return true;
    }

    // This version of __toMessage__ is used during initialization of
    // statics when __getParamDescriptions__ can't be called yet.
    void __toMessage__(dynamic_reconfigure::Config &msg, const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__, const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__) const
    {
      dynamic_reconfigure::ConfigTools::clear(msg);
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toMessage(msg, *this);

      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        if((*i)->id == 0)
        {
          (*i)->toMessage(msg, *this);
        }
      }
    }
    
    void __toMessage__(dynamic_reconfigure::Config &msg) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      __toMessage__(msg, __param_descriptions__, __group_descriptions__);
    }
    
    void __toServer__(const ros::NodeHandle &nh) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->toServer(nh, *this);
    }

    void __fromServer__(const ros::NodeHandle &nh)
    {
      static bool setup=false;

      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->fromServer(nh, *this);

      const std::vector<AbstractGroupDescriptionConstPtr> &__group_descriptions__ = __getGroupDescriptions__();
      for (std::vector<AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); i++){
        if (!setup && (*i)->id == 0) {
          setup = true;
          boost::any n = boost::any(this);
          (*i)->setInitialState(n);
        }
      }
    }

    void __clamp__()
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      const TrajectoryGeneratorUIConfig &__max__ = __getMax__();
      const TrajectoryGeneratorUIConfig &__min__ = __getMin__();
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->clamp(*this, __max__, __min__);
    }

    uint32_t __level__(const TrajectoryGeneratorUIConfig &config) const
    {
      const std::vector<AbstractParamDescriptionConstPtr> &__param_descriptions__ = __getParamDescriptions__();
      uint32_t level = 0;
      for (std::vector<AbstractParamDescriptionConstPtr>::const_iterator i = __param_descriptions__.begin(); i != __param_descriptions__.end(); ++i)
        (*i)->calcLevel(level, config, *this);
      return level;
    }
    
    static const dynamic_reconfigure::ConfigDescription &__getDescriptionMessage__();
    static const TrajectoryGeneratorUIConfig &__getDefault__();
    static const TrajectoryGeneratorUIConfig &__getMax__();
    static const TrajectoryGeneratorUIConfig &__getMin__();
    static const std::vector<AbstractParamDescriptionConstPtr> &__getParamDescriptions__();
    static const std::vector<AbstractGroupDescriptionConstPtr> &__getGroupDescriptions__();
    
  private:
    static const TrajectoryGeneratorUIConfigStatics *__get_statics__();
  };
  
  template <> // Max and min are ignored for strings.
  inline void TrajectoryGeneratorUIConfig::ParamDescription<std::string>::clamp(TrajectoryGeneratorUIConfig &config, const TrajectoryGeneratorUIConfig &max, const TrajectoryGeneratorUIConfig &min) const
  {
    return;
  }

  class TrajectoryGeneratorUIConfigStatics
  {
    friend class TrajectoryGeneratorUIConfig;
    
    TrajectoryGeneratorUIConfigStatics()
    {
TrajectoryGeneratorUIConfig::GroupDescription<TrajectoryGeneratorUIConfig::DEFAULT, TrajectoryGeneratorUIConfig> Default("Default", "", 0, 0, true, &TrajectoryGeneratorUIConfig::groups);
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.vel_trajectory = 0.5;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.vel_trajectory = 4.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.vel_trajectory = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("vel_trajectory", "double", 0, "Velocity in Trajectory Mode", "", &TrajectoryGeneratorUIConfig::vel_trajectory)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("vel_trajectory", "double", 0, "Velocity in Trajectory Mode", "", &TrajectoryGeneratorUIConfig::vel_trajectory)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.acc_trajectory = 0.5;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.acc_trajectory = 5.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.acc_trajectory = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("acc_trajectory", "double", 0, "Acceleration in Trajectory Mode", "", &TrajectoryGeneratorUIConfig::acc_trajectory)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("acc_trajectory", "double", 0, "Acceleration in Trajectory Mode", "", &TrajectoryGeneratorUIConfig::acc_trajectory)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.vel_kinematics = 0.5;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.vel_kinematics = 2.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.vel_kinematics = 1.0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("vel_kinematics", "double", 0, "Velocity in Kinematics Mode", "", &TrajectoryGeneratorUIConfig::vel_kinematics)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<double>("vel_kinematics", "double", 0, "Velocity in Kinematics Mode", "", &TrajectoryGeneratorUIConfig::vel_kinematics)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __min__.place_holder = 0;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __max__.place_holder = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __default__.place_holder = 1;
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.abstract_parameters.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<bool>("place_holder", "bool", 0, "-------------------------------------------------------------------------------", "", &TrajectoryGeneratorUIConfig::place_holder)));
//#line 259 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __param_descriptions__.push_back(TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr(new TrajectoryGeneratorUIConfig::ParamDescription<bool>("place_holder", "bool", 0, "-------------------------------------------------------------------------------", "", &TrajectoryGeneratorUIConfig::place_holder)));
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      Default.convertParams();
//#line 233 "/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py"
      __group_descriptions__.push_back(TrajectoryGeneratorUIConfig::AbstractGroupDescriptionConstPtr(new TrajectoryGeneratorUIConfig::GroupDescription<TrajectoryGeneratorUIConfig::DEFAULT, TrajectoryGeneratorUIConfig>(Default)));
//#line 390 "/opt/ros/indigo/share/dynamic_reconfigure/templates/ConfigType.h.template"

      for (std::vector<TrajectoryGeneratorUIConfig::AbstractGroupDescriptionConstPtr>::const_iterator i = __group_descriptions__.begin(); i != __group_descriptions__.end(); ++i)
      {
        __description_message__.groups.push_back(**i);
      }
      __max__.__toMessage__(__description_message__.max, __param_descriptions__, __group_descriptions__); 
      __min__.__toMessage__(__description_message__.min, __param_descriptions__, __group_descriptions__); 
      __default__.__toMessage__(__description_message__.dflt, __param_descriptions__, __group_descriptions__); 
    }
    std::vector<TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr> __param_descriptions__;
    std::vector<TrajectoryGeneratorUIConfig::AbstractGroupDescriptionConstPtr> __group_descriptions__;
    TrajectoryGeneratorUIConfig __max__;
    TrajectoryGeneratorUIConfig __min__;
    TrajectoryGeneratorUIConfig __default__;
    dynamic_reconfigure::ConfigDescription __description_message__;

    static const TrajectoryGeneratorUIConfigStatics *get_instance()
    {
      // Split this off in a separate function because I know that
      // instance will get initialized the first time get_instance is
      // called, and I am guaranteeing that get_instance gets called at
      // most once.
      static TrajectoryGeneratorUIConfigStatics instance;
      return &instance;
    }
  };

  inline const dynamic_reconfigure::ConfigDescription &TrajectoryGeneratorUIConfig::__getDescriptionMessage__() 
  {
    return __get_statics__()->__description_message__;
  }

  inline const TrajectoryGeneratorUIConfig &TrajectoryGeneratorUIConfig::__getDefault__()
  {
    return __get_statics__()->__default__;
  }
  
  inline const TrajectoryGeneratorUIConfig &TrajectoryGeneratorUIConfig::__getMax__()
  {
    return __get_statics__()->__max__;
  }
  
  inline const TrajectoryGeneratorUIConfig &TrajectoryGeneratorUIConfig::__getMin__()
  {
    return __get_statics__()->__min__;
  }
  
  inline const std::vector<TrajectoryGeneratorUIConfig::AbstractParamDescriptionConstPtr> &TrajectoryGeneratorUIConfig::__getParamDescriptions__()
  {
    return __get_statics__()->__param_descriptions__;
  }

  inline const std::vector<TrajectoryGeneratorUIConfig::AbstractGroupDescriptionConstPtr> &TrajectoryGeneratorUIConfig::__getGroupDescriptions__()
  {
    return __get_statics__()->__group_descriptions__;
  }

  inline const TrajectoryGeneratorUIConfigStatics *TrajectoryGeneratorUIConfig::__get_statics__()
  {
    const static TrajectoryGeneratorUIConfigStatics *statics;
  
    if (statics) // Common case
      return statics;

    boost::mutex::scoped_lock lock(dynamic_reconfigure::__init_mutex__);

    if (statics) // In case we lost a race.
      return statics;

    statics = TrajectoryGeneratorUIConfigStatics::get_instance();
    
    return statics;
  }


}

#endif // __TRAJECTORYGENERATORUIRECONFIGURATOR_H__