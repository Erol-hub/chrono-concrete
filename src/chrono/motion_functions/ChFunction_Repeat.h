// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHFUNCT_REPEAT_H
#define CHFUNCT_REPEAT_H

#include "chrono/motion_functions/ChFunction_Base.h"
#include "chrono/motion_functions/ChFunction_Const.h"

namespace chrono {

/// @addtogroup chrono_functions
/// @{

/// Repeat function:
///     `y = __/__/__/`
///
/// Repeats a 'window' of a function, periodically.
/// Evaluates a fa(t) function as:
///
///   fa(t) = fa(window_start + mod(t + window_phase, window_length))
///
/// Note: for infinite window_length and zero window_start, you can use
/// window_phase to simply 'translate' the function on abscissa.
class ChApi ChFunction_Repeat : public ChFunction {
  public:
    ChFunction_Repeat(std::shared_ptr<ChFunction> func, double start = 0, double length = 1, double phase = 0);
    ChFunction_Repeat(const ChFunction_Repeat& other);
    ~ChFunction_Repeat() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChFunction_Repeat* Clone() const override { return new ChFunction_Repeat(*this); }

    virtual FunctionType Get_Type() const override { return FUNCT_REPEAT; }

    virtual double Get_y(double x) const override;

    void Set_window_start(double start) { window_start = start; }
    double Get_window_start() const { return window_start; }

    void Set_window_length(double length) { window_length = length; }
    double Get_window_length() const { return window_length; }

    void Set_window_phase(double phase) { window_phase = phase; }
    double Get_window_phase() const { return window_phase; }

    void Set_fa(std::shared_ptr<ChFunction> func) { fa = func; }
    std::shared_ptr<ChFunction> Get_fa() { return fa; }

    virtual void Estimate_x_range(double& xmin, double& xmax) const override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive) override;

  private:
    double window_start;   ///< window begin position
    double window_length;  ///< window length
    double window_phase;   ///< window phase
    std::shared_ptr<ChFunction> fa;
};

/// @} chrono_functions

CH_CLASS_VERSION(ChFunction_Repeat, 0)

}  // end namespace chrono

#endif
