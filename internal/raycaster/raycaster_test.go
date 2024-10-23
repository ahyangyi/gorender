package raycaster

import (
	"github.com/mattkimber/gandalf/magica"
	"github.com/ahyangyi/gorender/internal/colour"
	"github.com/ahyangyi/gorender/internal/geometry"
	"github.com/ahyangyi/gorender/internal/manifest"
	"github.com/ahyangyi/gorender/internal/sampler"
	"github.com/ahyangyi/gorender/internal/voxelobject"
	"testing"
)

func Test_getLightingValue(t *testing.T) {
	testCases := []struct {
		normal, lighting geometry.Vector3
		expected         float64
	}{
		{geometry.Vector3{}, geometry.UnitX(), 0.0},
		{geometry.UnitX(), geometry.UnitX(), 1.0},
		{geometry.Vector3{X: 0.5, Y: 1}.Normalise(), geometry.Vector3{X: 1, Y: 0.5, Z: 1}.Normalise(), 0.5962847939999438},
	}
	for _, testCase := range testCases {
		if result := getLightingValue(testCase.normal, testCase.lighting); result != testCase.expected {
			t.Errorf("getLightingValue for normal %v and lighting %v returned %v, expected %v", testCase.normal, testCase.lighting, result, testCase.expected)
		}
	}
}

func Test_raycaster(t *testing.T) {
	object := getObject("cone.vox", t)
	m := manifest.Manifest{
		LightingAngle:        45,
		LightingElevation:    50,
		Size:                 object.Size.ToVector3(),
		RenderElevationAngle: 30,
		Sprites:              []manifest.Sprite{{Angle: 45, Width: 10, Height: 10}},
		DepthInfluence:       0.1,
		TiledNormals:         false,
		SoftenEdges:          0,
	}

	smp := sampler.Square(100, 100, 2, 0, 0)
	_ = GetRaycastOutput(object, m, m.Sprites[0], smp)

}

func getObject(filename string, t *testing.T) voxelobject.ProcessedVoxelObject {
	mv, err := magica.FromFile("testdata/" + filename)
	if err != nil {
		t.Fatalf("error loading test file: %v", err)
	}

	entries := make([]colour.PaletteEntry, 256)

	pal := colour.Palette{
		Entries:                           entries,
		CompanyColourLightingContribution: 0,
		DefaultBrightness:                 0,
		CompanyColourLightingScale:        0,
	}

	pal.SetRanges([]colour.PaletteRange{{Start: 0, End: 255}})

	v := voxelobject.GetProcessedVoxelObject(mv, &pal, false, "normal", false)
	return v
}

func Benchmark_castFpRay(b *testing.B) {
	object := getObjectForBenchmark("cone.vox", b)
	size := object.Size
	limits := geometry.Vector3{X: float64(size.X), Y: float64(size.Y), Z: float64(size.Z)}

	ray := geometry.Vector3{X: -1, Y: 0, Z: -0.125}.Normalise()
	loc := geometry.Vector3{X: 80, Y: 20, Z: 30}

	for i := 0; i < b.N; i++ {
		_ = castFpRay(object, loc, loc, ray, limits, false)
	}
}

func Benchmark_raycaster(b *testing.B) {
	object := getObjectForBenchmark("cone.vox", b)
	m := manifest.Manifest{
		LightingAngle:        45,
		LightingElevation:    50,
		Size:                 object.Size.ToVector3(),
		RenderElevationAngle: 30,
		Sprites:              []manifest.Sprite{{Angle: 45, Width: 10, Height: 10}},
		DepthInfluence:       0.1,
		TiledNormals:         false,
		SoftenEdges:          0,
	}

	smp := sampler.Square(50, 50, 2, 0, 0)
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = GetRaycastOutput(object, m, m.Sprites[0], smp)
	}
}

func getObjectForBenchmark(filename string, b *testing.B) voxelobject.ProcessedVoxelObject {
	mv, err := magica.FromFile("testdata/" + filename)
	if err != nil {
		b.Fatalf("error loading test file: %v", err)
	}

	v := voxelobject.GetProcessedVoxelObject(mv, &colour.Palette{}, false, "normal", false)
	return v
}
