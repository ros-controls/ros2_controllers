/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Mark Naeem
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
 *   * The names of the contributors may NOT be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
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
 *********************************************************************/

/*
 * Author: Mark Naeem
 */

#include <swerve_steering_controller/interval.h>

interval::interval(const std::array<double, 2> &limits, const std::string &type)
{
    const auto& types_val = types_dict_[type];
    this->limits_.push_back(limits);
    this->types_.push_back(types_val);
}


bool interval::intersect(std::array<double,2>& limits1, const std::array<double,2>& limits2, std::array<int,2>& type1, const std::array<int,2>& type2)
{
    bool interval1_point, interval2_point = false;
    if (limits1[0] == limits1[1]) interval1_point = true;
    if (limits2[0] == limits2[1]) interval2_point = true;

    if (interval1_point && interval2_point) 
        return (limits1[0]==limits2[0])? true: false;

    else if (interval1_point && !interval2_point)
    {
        if (limits1[0]==limits2[0]) 
            return (type2[0]==0)?false:true;
        else if (limits1[0]==limits2[1]) 
            return (type2[1]==0)?false:true;
        else if (limits1[0]>limits2[0] && limits1[0]<limits2[1]) return true;
        else return false;
    }
    
    else if (!interval1_point && interval2_point)
    {
        if (limits2[0]==limits1[0]) 
            return (type1[0]==0)?false:true;
        else if (limits2[0]==limits1[1]) 
            return (type1[1]==0)?false:true;
        else if (limits2[0]>limits1[0] && limits2[0]<limits1[1]) return true;
        else return false;
    }
    
    else
    {
        limits_types_intervals_vector_.clear();
        limits_types_intervals_vector_.push_back({limits1[0],type1[0],0});
        limits_types_intervals_vector_.push_back({limits1[1],type1[1],0});
        limits_types_intervals_vector_.push_back({limits2[0],type2[0],1});
        limits_types_intervals_vector_.push_back({limits2[1],type2[1],1});

        utils::limits_types_intervals::sort(limits_types_intervals_vector_);                  
                  
        if (limits_types_intervals_vector_[1].limit==limits_types_intervals_vector_[2].limit &&
                  limits_types_intervals_vector_[1].interval!=limits_types_intervals_vector_[2].interval)
                  if (limits_types_intervals_vector_[1].type==0 || limits_types_intervals_vector_[2].type==0) return false;
                  else return true;
        else if ((limits_types_intervals_vector_[0].interval==0 &&
         limits_types_intervals_vector_[1].interval==0 &&
         limits_types_intervals_vector_[2].interval==1 &&
         limits_types_intervals_vector_[3].interval==1)
         ||
        (limits_types_intervals_vector_[0].interval==1 &&
         limits_types_intervals_vector_[1].interval==1 &&
         limits_types_intervals_vector_[2].interval==0 &&
         limits_types_intervals_vector_[3].interval==0))
         return false;

        else
            return true;
    }

}


bool interval::is_intersecting(const interval& another_interval)
{
    const auto size1  = this->limits_.size();
    const auto size2  = another_interval.limits_.size();

    if (size1==1 && size2==1)
        return intersect(limits_[0], another_interval.limits_[0], types_[0], another_interval.types_[0]);
    else if (size1==1 && size2==2)
    {
        const auto& b1 =  intersect(limits_[0], another_interval.limits_[0], types_[0], another_interval.types_[0]);
        const auto& b2 =  intersect(limits_[0], another_interval.limits_[1], types_[0], another_interval.types_[1]);
        return b1 || b2;
    }
    else if (size1==2 && size2==1)
    {
        const auto& b1 =  intersect(limits_[0], another_interval.limits_[0], types_[0], another_interval.types_[0]);
        const auto& b2 =  intersect(limits_[1], another_interval.limits_[0], types_[1], another_interval.types_[0]);
        return b1 || b2;
    }
    else
    {
        const auto& b1 =  intersect(limits_[0], another_interval.limits_[0], types_[0], another_interval.types_[0]);
        const auto& b2 =  intersect(limits_[1], another_interval.limits_[0], types_[1], another_interval.types_[0]);
        const auto& b3 =  intersect(limits_[0], another_interval.limits_[1], types_[0], another_interval.types_[1]);
        const auto& b4 =  intersect(limits_[1], another_interval.limits_[1], types_[1], another_interval.types_[1]);
        return b1 || b2 || b3 || b4;
    }
}

interval interval::complement () const
{
    /*if (limits_.size()>1) //do something  because we can't do it yet*/
    interval complementary_interval(std::array<double,2>{-(M_PI+0.1),limits_[0][0]}, types_[0][0]==1 ? "closeopen" : "close"); 
    complementary_interval.limits_.push_back(std::array<double,2>{limits_[0][1],(M_PI+0.1)});
    complementary_interval.types_.push_back(types_[0][1]==1 ? std::array<int,2>{0,1} : std::array<int,2>{1,1});
    return complementary_interval;
}

double interval::len() const
{
    double res = 0;
    for (auto it: limits_) // we need this to be a copy not a reference
    {
        if (it[0]==-(M_PI+0.1))  it[0] =-M_PI;
        if (it[1]== (M_PI+0.1))   it[1] = M_PI;
        res += fabs(it[0]-it[1]);
    }
    return res;
}

void interval::print()const
{
    for (const auto& it: limits_)
    {
        std::cout<<"[ "<<it[0]<<" , "<<it[1]<<" ]";
    }
    std::cout<<std::endl;
    for (const auto& it: types_)
    {
        std::cout<<"[ "<<it[0]<<" , "<<it[1]<<" ]";
    }
    std::cout<<std::endl;
}