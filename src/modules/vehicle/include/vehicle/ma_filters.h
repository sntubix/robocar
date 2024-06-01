/*
 * MIT License
 * Copyright (c) 2024 University of Luxembourg
 */

#ifndef VEHICLE_MA_FILTERS_H
#define VEHICLE_MA_FILTERS_H

#include <cassert>
#include <cmath>

namespace robocar::vehicle
{
	class MA
	{
	public:
		explicit MA(unsigned int period)
			: period(period), window(new double[period]), alpha(2.0 / static_cast<double>(period + 1.0))
		{
			assert(period >= 1);
		}

		~MA()
		{
			delete[] window;
		}

		// Adds a value to the average, pushing one out if nescessary
		void add(double val)
		{
			// Special case: Initialization
			if (head == nullptr)
			{
				head = window;
				*head = val;
				tail = head;
				inc(tail);
				total = val;
				ema_total = val;
				dema_total = val;
				tema_total = val;
				return;
			}

			// Were we already full?
			if (head == tail)
			{
				// Fix total-cache
				total -= *head;
				// Make room
				inc(head);
			}

			// Write the value in the next spot.
			*tail = val;
			inc(tail);

			// Update our total-cache
			total += val;
			sma_avg_std_dev = std::pow(val - SMA_avg(), 2);
			ema_avg_std_dev = std::pow(val - EMA_avg(), 2);
			ema_total = (val * alpha) + (ema_total * (1 - alpha));
			dema_total = 2 * ((val * alpha) + (ema_total * (1 - alpha))) - dema_total;
			tema_total = 3 * ((val * alpha) + (ema_total * (1 - alpha))) -
						 3 * (2 * ((val * alpha) + (ema_total * (1 - alpha))) - dema_total) + tema_total;
		}

		double sum() const
		{
			return total;
		}

		// Returns the average of the last P elements added to this SMA.
		// If no elements have been added yet, returns 0.0
		double SMA_avg() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			return total / (double)size; // Cast to double for floating point arithmetic
		}

		// Returns the standard deviation of the last P elements added to the SMA.
		double SMA_avg_STD_dev() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			if (size == 1)
			{
				return std::sqrt(sma_avg_std_dev / (double)size); // Fisrt entries
			}
			return std::sqrt(sma_avg_std_dev / (double)(size - 1));
		}

		// Returns the exponential moving average
		double EMA_avg() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			return ema_total;
		}

		// Returns the standard deviation of the last P elements added to the EMA.
		double EMA_avg_STD_dev() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			if (size == 1)
			{
				return std::sqrt(ema_avg_std_dev / (double)size); // Fisrt entries
			}
			return std::sqrt(ema_avg_std_dev / (double)(size - 1));
		}

		// Returns the Double exponential moving average
		double DEMA_avg() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			return dema_total;
		}

		// Returns the Triple exponential moving average
		double TEMA_avg() const
		{
			ptrdiff_t size = this->size();
			if (size == 0)
			{
				return 0; // No entries => 0 average
			}
			return tema_total;
		}

		// Returns how many numbers we have stored.
		ptrdiff_t size() const
		{
			if (head == nullptr)
				return 0;
			if (head == tail)
				return period;
			return (period + tail - head) % period;
		}

		void clear()
		{
			head = nullptr;
			tail = nullptr;
			total = 0.0;
			sma_avg_std_dev = 0.0;
			ema_avg_std_dev = 0.0;
		}

	private:
		unsigned int period = 0;
		double *window = nullptr; // Holds the values to calculate the average of.

		// Logically, head is before tail
		double *head = nullptr; // Points at the oldest element we've stored.
		double *tail = nullptr; // Points at the newest element we've stored.

		double total = 0.0, sma_avg_std_dev = 0.0, ema_avg_std_dev = 0.0; // Cache the total so we don't sum everything each time.

		const double alpha;
		double ema_total = 0.0, dema_total = 0.0, tema_total = 0.0; // Cache the Exponential total, Double total, and Triple total.

		// Bumps the given pointer up by one.
		// Wraps to the start of the array if needed.
		void inc(double *&p)
		{
			if (++p >= window + period)
			{
				p = window;
			}
		}
	};

	class MA_Angle
	{
	public:
		explicit MA_Angle(unsigned int period) : ma_cos_(period), ma_sin_(period)
		{
			assert(period >= 1);
		}

		void add(double a)
		{
			ma_sin_.add(std::sin(a));
			ma_cos_.add(std::cos(a));
		}

		double avg()
		{
			double r = std::atan2(ma_sin_.sum(), ma_cos_.sum());
			if (r < 0.0)
			{
				r += M_PI * 2;
			}
			return r;
		}

		void clear()
		{
			ma_sin_.clear();
			ma_cos_.clear();
		}

	private:
		MA ma_cos_;
		MA ma_sin_;
	};
}

#endif // VEHICLE_MA_FILTERS_H
