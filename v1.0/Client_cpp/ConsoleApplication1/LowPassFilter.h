#ifndef LowPassFilter_h
#define LowPassFilter_h

template <class T>
class LowPassFilter
{
public:
	LowPassFilter(int order, T *denominator, T *numerator, T initialValue);
	~LowPassFilter();
	T update(T data);
	T getFilteredData();

private:
	int n;
	T* a;
	T* b;
	T* x;
	T* y;
	void push(T* arr);
	bool iscomputing = false;
};

#endif
