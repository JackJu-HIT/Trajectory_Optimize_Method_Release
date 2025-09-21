#pragma once 
namespace tools
{
    inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
    {
        if (var >= a+epsilon)
        {
            return 0.;
        }
        else
        {
            return (-var + (a+epsilon));
        }
    }
    /**
     * @brief Linear penalty function for bounding \c var to the interval \f$ -a < var < a \f$
     * @param var The scalar that should be bounded
     * @param a lower and upper absolute bound
     * @param epsilon safty margin (move bound to the interior of the interval)
     * @see penaltyBoundToIntervalDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundToInterval(const double& var,const double& a,const double& epsilon)
    {
        if (var < -a+epsilon)
        {
            return (-var - (a - epsilon));
        }
        if (var <= a-epsilon)
        {
            return 0.;
        }
        else
        {
            return (var - (a - epsilon));
        }
    }


    /**
     * @brief Linear penalty function for bounding \c var to the interval \f$ a < var < b \f$
     * @param var The scalar that should be bounded
     * @param a lower bound
     * @param b upper bound
     * @param epsilon safty margin (move bound to the interior of the interval)
     * @see penaltyBoundToIntervalDerivative
     * @return Penalty / cost value that is nonzero if the constraint is not satisfied
     */
    inline double penaltyBoundToInterval(const double& var,const double& a, const double& b, const double& epsilon)
    {
    if (var < a+epsilon)
    {
        return (-var + (a + epsilon));
    }
    if (var <= b-epsilon)
    {
        return 0.;
    }
    else
    {
        return (var - (b - epsilon));
    }
    }


    struct pathInfo
    {
        float x;
        float y;
        float theta;
    };

    struct obstacleInfo
    {
        float x;
        float y;
        float theta;
    };

    inline double distanceBetweenTwoPoint(pathInfo& a,pathInfo& b)
    {
        double dist = sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
        return dist;
    }

    inline double normalize_theta(double theta)
    {
        if (theta >= -3.14 && theta < 3.14)
            return theta;
        
        double multiplier = std::floor(theta / (2*3.14));
        theta = theta - multiplier*2*3.14;
        if (theta >=3.14)
            theta -= 2*3.14;
        if (theta < -3.14)
            theta += 2*3.14;

        return theta;
    }
}