package raycaster

import (
	"github.com/ahyangyi/gorender/internal/geometry"
	"github.com/ahyangyi/gorender/internal/manifest"
	"math"
)

func getRenderDirection(angle float64, elevationAngle float64) geometry.Vector3 {
	x, y, z := -math.Cos(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(elevationAngle))
	return geometry.Vector3{X: x, Y: y, Z: z}.Normalise()
}

func getLightingDirection(angle float64, elevation float64, flipY bool) geometry.Vector3 {
	x, y, z := -math.Cos(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(elevation))
	if flipY {
		y = -y
	}
	return geometry.Zero().Subtract(geometry.Vector3{X: x, Y: y, Z: z}).Normalise()
}

func getViewportPlane(angle float64, m manifest.Manifest, zError float64, size geometry.Point, elevationAngle float64) geometry.Plane {
	cos, sin := math.Cos(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(angle))

	midpointX := float64(size.X) / 2.0
	if m.PadToFullLength {
		midpointX -= ((m.Size.X) - float64(size.X)) / 2.0
	}

	midpoint := geometry.Vector3{X: midpointX, Y: float64(size.Y) / 2.0, Z: (m.Size.Z - zError) / 2.0}

	direction := getRenderDirection(angle, elevationAngle)
	viewpoint := midpoint.Add(direction.MultiplyByConstant(m.Size.X))

	planeNormalXComponent := math.Abs(((m.Size.X) / 2.0) * cos * math.Sin(geometry.DegToRad(elevationAngle)))
	planeNormalYComponent := math.Abs(((m.Size.Y) / 2.0) * sin * math.Sin(geometry.DegToRad(elevationAngle)))
	planeNormalZComponent := m.Size.Z / 2.0

	constant := planeNormalXComponent + planeNormalYComponent + planeNormalZComponent
	constant = constant * (1.0 + zError)
	planeNormal := geometry.UnitZ().MultiplyByConstant(constant)

	renderNormalXComponent := math.Abs(((m.Size.X) / 2.0) * sin)
	renderNormalYComponent := math.Abs(((m.Size.Y) / 2.0) * cos)
	renderNormal := getRenderNormal(angle).MultiplyByConstant(renderNormalXComponent + renderNormalYComponent)

	a := viewpoint.Subtract(renderNormal).Subtract(planeNormal)
	b := viewpoint.Add(renderNormal).Subtract(planeNormal)
	c := viewpoint.Add(renderNormal).Add(planeNormal)
	d := viewpoint.Subtract(renderNormal).Add(planeNormal)

	return geometry.Plane{A: a, B: b, C: c, D: d}
}

func getRenderNormal(angle float64) geometry.Vector3 {
	x, y := -math.Cos(geometry.DegToRad(angle)), math.Sin(geometry.DegToRad(angle))
	return geometry.Vector3{X: y, Y: -x}.Normalise()
}
