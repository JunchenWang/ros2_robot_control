#ifndef MOVINGFILTER_H
#define MOVINGFILTER_H

#include <queue>
#include <vector>
#include <cstring>
namespace robot_math
{
    template <class T>
    class MovingFilter
    {
    public:
        MovingFilter(int nChannel, int N = 50);
        ~MovingFilter();
        void filtering(const T *datain, T *dataout);
        void reset();

    private:
        int Width;
        int Channel;
        int n;
        std::vector<T> sum;
        std::queue<T> qu;
    };

    template <class T>
    MovingFilter<T>::MovingFilter(int nChannel, int N) : sum(nChannel)
    {
        Channel = nChannel;
        Width = N;
        memset(&sum[0], 0, nChannel * sizeof(T));
    }

    template <class T>
    void MovingFilter<T>::reset()
    {
        std::queue<T> empty;
        memset(&sum[0], 0, Channel * sizeof(T));
        std::swap(qu, empty);
        // qu.clear();
    }

    template <class T>
    MovingFilter<T>::~MovingFilter()
    {
    }

    template <class T>
    void MovingFilter<T>::filtering(const T *datain, T *dataout)
    {

        for (int i = 0; i < Channel; i++)
            qu.push(datain[i]), sum[i] += datain[i];

        if (qu.size() > Width * Channel)
            for (int i = 0; i < Channel; i++)
                sum[i] -= qu.front(), qu.pop();

        int n = qu.size() / Channel;
        for (int i = 0; i < Channel; i++)
            dataout[i] = sum[i] / n;
    }
}
#endif