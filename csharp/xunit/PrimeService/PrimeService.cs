using System;

namespace Prime.Service
{
    public class PrimeService
    {
        public bool IsPrime(int candidate)
        {
            if (candidate < 2)
            {
                return false;
            }
            if (candidate == 2)
            {
                return true;
            }
            if (candidate % 2 == 0)
            {
                return false;
            }
            for (int i = 3; i*i <= candidate; i += 2)
            {
                if (candidate % i == 0)
                {
                    return false;
                }
            }
            return true;
        }
    }
}
