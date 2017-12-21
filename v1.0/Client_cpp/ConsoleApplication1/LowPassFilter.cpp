#include "LowPassFilter.h"

template <class T>
LowPassFilter<T>::LowPassFilter(int order, T *denominator, T *numerator, T initialValue)
{
	n = order;
	a = new T[n + 1];
	b = new T[n + 1];
	x = new T[n + 1];
	y = new T[n + 1];
	for (int i = 0; i < n + 1; i++)
	{
		a[i] = denominator[i];
		b[i] = numerator[i];
		x[i] = initialValue;
		y[i] = initialValue;
	}
}

template <class T>
LowPassFilter<T>::~LowPassFilter()
{
	delete[] a;
	delete[] b;
	delete[] x;
	delete[] y;
}

template <class T>
T LowPassFilter<T>::update(T data)
{
	iscomputing = true;
	push(x);
	x[0] = data;
	push(y);
	y[0] = 0;

	// Infinite Impulse Response (IIR) filter
	for (int i = 0; i < n + 1; i++) y[0] += b[i]*x[i];
	for (int i = 1; i < n + 1; i++) y[0] -= a[i]*y[i];
	iscomputing = false;

	return y[0];
}

template <class T>
T LowPassFilter<T>::getFilteredData()
{
	while (iscomputing);
	return y[0];
}

template <class T>
void LowPassFilter<T>::push(T* arr)
{
	for (int i = n; i > 0; i--) arr[i] = arr[i - 1];
}