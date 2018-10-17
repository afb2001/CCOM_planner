package dubins

import "math"

// This is a direct port of dubins.cpp by Mostafa.
const EPSILON = 10e-10

type SegmentType int

const (
	L_SEG SegmentType = iota
	S_SEG
	R_SEG
)

var DIRDATA = [6][3]SegmentType{
	{L_SEG, S_SEG, L_SEG},
	{L_SEG, S_SEG, R_SEG},
	{R_SEG, S_SEG, L_SEG},
	{R_SEG, S_SEG, R_SEG},
	{R_SEG, L_SEG, R_SEG},
	{L_SEG, R_SEG, L_SEG},
}

const (
	EDUBOK        = iota // No error
	EDUBCOCONFIGS        // Colocated configurations
	EDUBPARAM            // path parameterization error
	EDUBBADRHO           // rho value is invalid
	EDUBNOPATH           // no connection between configurations with this word
	EDUBOTHER            // other error
)

type PathType int

const (
	LSL PathType = iota
	LSR
	RSL
	RSR
	RLR
	LRL
)

type Path struct {
	// initial configuration
	qi [3]float64
	// lengths of three segments
	param [3]float64
	// forward velocity / angular velocity
	rho float64
	// path type
	pathType PathType
}

type IntermediateResults struct {
	alpha, beta, d, sa, sb, ca, cb, c_ab, d_sq float64
}

type PathSamplingCallback func(q [3]float64, t float64, userData interface{}) int

func fmodr(x float64, y float64) float64 {
	return x - y*math.Floor(x/y)
}

func mod2pi(theta float64) float64 {
	return fmodr(theta, 2*math.Pi)
}

func ShortestPath(path *Path, q0 [3]float64, q1 [3]float64, rho float64) int {
	var params [3]float64
	var cost float64
	bestCost := math.MaxFloat64
	bestWord := -1

	in, err := MakeIntermediateResults(q0, q1, rho)

	if err != EDUBOK {
		return err
	}

	path.qi[0], path.qi[1], path.qi[2] = q0[0], q0[1], q0[2]
	path.rho = rho

	for i := 0; i < 6; i++ {
		pathType := PathType(i)
		params, err = BuildWord(in, pathType)
		if err == EDUBOK {
			cost = params[0] + params[1] + params[2]
			if cost < bestCost {
				bestWord = i
				bestCost = cost
				path.param[0], path.param[1], path.param[2] =
					params[0], params[1], params[2]
				path.pathType = pathType
			}
		}
	}
	if bestWord == -1 {
		return EDUBNOPATH
	}
	return EDUBOK
}

func CreatePath(path *Path, q0 [3]float64, q1 [3]float64, rho float64, pathType PathType) int {
	in, err := MakeIntermediateResults(q0, q1, rho)
	if err == EDUBOK {
		var params [3]float64
		params, err = BuildWord(in, pathType)
		if err == EDUBOK {
			path.param[0], path.param[1], path.param[2] =
				params[0], params[1], params[2]
			path.qi[0], path.qi[1], path.qi[2] = q0[0], q0[1], q0[2]
			path.rho = rho
			path.pathType = pathType
		}
	}
	return err
}

func (p Path) Length() float64 {
	return (p.param[0] + p.param[1] + p.param[2]) * p.rho
}

func (p Path) SegmentLengthNormalized(i int) float64 {
	if i < 0 || i > 2 {
		return math.MaxFloat64
	}
	return p.param[i]
}

func (p Path) SegmentLength(i int) float64 {
	if i < 0 || i > 2 {
		return math.MaxFloat64
	}
	return p.SegmentLengthNormalized(i) * p.rho
}

func Segment(t float64, qi [3]float64, qt [3]float64, segmentType SegmentType) {
	st, ct := math.Sin(qi[2]), math.Cos(qi[2])
	switch segmentType {
	case L_SEG:
		qt[0] = math.Sin(qi[2]+t) - st
		qt[1] = -math.Cos(qi[2]+t) + ct
		qt[2] = t
		break
	case R_SEG:
		qt[0] = -math.Sin(qi[2]-t) + st
		qt[1] = math.Cos(qi[2]-t) - ct
		qt[2] = -t
		break
	case S_SEG:
		qt[0] = ct * t
		qt[1] = st * t
		qt[2] = 0
		break
	}
	qt[0] += qi[0]
	qt[1] += qi[1]
	qt[2] += qi[2]
}

func (p *Path) Sample(t float64, q [3]float64) int {
	tPrime := t / p.rho
	var qi, q1, q2 [3]float64
	var types = DIRDATA[p.pathType]

	if t < 0 || t > p.Length() {
		return EDUBPARAM
	}

	qi[0], qi[1], qi[2] = 0, 0, p.qi[2]

	p1, p2 := p.param[0], p.param[1]

	Segment(p1, qi, q1, types[0])
	Segment(p2, q1, q2, types[1])

	if tPrime < p1 {
		Segment(tPrime, qi, q, types[0])
	} else if tPrime < p1+p2 {
		Segment(tPrime-p1, q1, q, types[1])
	} else {
		Segment(tPrime-p1-p2, q2, q, types[2])
	}

	q[0] = q[0]*p.rho + p.qi[0]
	q[1] = q[1]*p.rho + p.qi[1]
	q[2] = mod2pi(q[2])

	return EDUBOK
}

func (p *Path) SampleMany(stepSize float64,
	cb PathSamplingCallback, userData interface{}) int {
	x, length := 0.0, p.Length()
	var q [3]float64
	for ; x < length; x += stepSize {
		p.Sample(x, q)
		if ret := cb(q, x, userData); ret != 0 {
			return ret
		}
	}
	return EDUBOK
}

func (p *Path) EndPoint(q [3]float64) int {
	return p.Sample(p.Length()-EPSILON, q)
}

func (p *Path) ExtractSubpath(t float64, newPath *Path) int {
	tprime := t / p.rho
	if t < 0 || t > p.Length() {
		return EDUBPARAM
	}

	newPath.qi[0], newPath.qi[1], newPath.qi[2], newPath.rho, newPath.pathType =
		p.qi[0], p.qi[1], p.qi[2], p.rho, p.pathType

	newPath.param[0] = math.Min(p.param[0], tprime)
	newPath.param[1] = math.Min(p.param[1], tprime-newPath.param[0])
	newPath.param[2] = math.Min(p.param[2], tprime-newPath.param[0]-newPath.param[1])

	return EDUBOK
}

func MakeIntermediateResults(q0 [3]float64, q1 [3]float64, rho float64) (results *IntermediateResults, err int) {
	if rho <= 0 {
		return nil, EDUBBADRHO
	}

	dx, dy := q1[0]-q0[0], q1[1]-q0[1]
	d := math.Sqrt(dx*dx+dy*dy) / rho

	var theta float64

	if d > 0 {
		theta = mod2pi(math.Atan2(dy, dx))
	}

	alpha, beta := mod2pi(q0[2]-theta), mod2pi(q1[2]-theta)

	return &IntermediateResults{
		alpha: alpha, beta: beta, d: d,
		sa: math.Sin(alpha), sb: math.Sin(beta),
		ca: math.Cos(alpha), cb: math.Cos(beta),
		c_ab: math.Cos(alpha - beta), d_sq: d * d,
	}, EDUBOK
}

func BuildLSL(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := in.d + in.sa - in.sb
	p_sq := 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sa - in.sb))
	if p_sq >= 0 {
		tmp1 := math.Atan2(in.cb-in.ca, tmp0)
		return [3]float64{mod2pi(tmp1 - in.alpha), math.Sqrt(p_sq), mod2pi(in.beta - tmp1)}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildRSR(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := in.d - in.sa + in.sb
	p_sq := 2 + in.d_sq - (2 * in.c_ab) + (2 * in.d * (in.sb - in.sa))
	if p_sq >= 0 {
		tmp1 := math.Atan2(in.ca-in.cb, tmp0)
		return [3]float64{mod2pi(in.alpha - tmp1), math.Sqrt(p_sq), mod2pi(tmp1 - in.beta)}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildLSR(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := in.d + in.sa + in.sb
	p_sq := -2 + in.d_sq + (2 * in.c_ab) + (2 * in.d * (in.sa + in.sb))
	if p_sq >= 0 {
		tmp1 := math.Atan2(-in.ca-in.cb, tmp0) - math.Atan2(-2, math.Sqrt(p_sq))
		return [3]float64{mod2pi(tmp1 - in.alpha), math.Sqrt(p_sq), mod2pi(tmp1 - in.beta)}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildRSL(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := in.d - in.sa - in.sb
	p_sq := -2 + in.d_sq + (2 * in.c_ab) - (2 * in.d * (in.sa + in.sb))
	if p_sq >= 0 {
		tmp1 := math.Atan2(in.ca+in.cb, tmp0) - math.Atan2(-2, math.Sqrt(p_sq))
		return [3]float64{mod2pi(in.alpha - tmp1), math.Sqrt(p_sq), mod2pi(in.beta - tmp1)}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildRLR(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := (6. - in.d_sq + 2*in.c_ab + 2*in.d*(in.sa-in.sb)) / 8.
	phi := math.Atan2(in.ca-in.cb, in.d-in.sa+in.sb)
	if math.Abs(tmp0) <= 1 {
		p := mod2pi((2 * math.Pi) - math.Acos(tmp0))
		t := mod2pi(in.alpha - phi + mod2pi(p/2.))
		return [3]float64{t, p, mod2pi(in.alpha - in.beta - t + mod2pi(p))}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildLRL(in *IntermediateResults) (out [3]float64, err int) {
	tmp0 := (6. - in.d_sq + 2*in.c_ab + 2*in.d*(in.sb-in.sa)) / 8.
	phi := math.Atan2(in.ca-in.cb, in.d+in.sa-in.sb)
	if math.Abs(tmp0) <= 1 {
		p := mod2pi((2 * math.Pi) - math.Acos(tmp0))
		t := mod2pi(-in.alpha - phi + mod2pi(p/2.))
		return [3]float64{t, p, mod2pi(mod2pi(in.beta) - in.alpha - t + mod2pi(p))}, EDUBOK
	}
	return out, EDUBNOPATH
}

func BuildWord(in *IntermediateResults, pathType PathType) (out [3]float64, err int) {
	switch pathType {
	case LSL:
		return BuildLSL(in)
	case RSL:
		return BuildRSL(in)
	case LSR:
		return BuildLSR(in)
	case RSR:
		return BuildRSR(in)
	case LRL:
		return BuildLRL(in)
	case RLR:
		return BuildRLR(in)
	}
	return out, EDUBOTHER
}
