package colour

import (
	"image/color"
	"strings"
	"testing"
)

const exampleJson = "{\"company_colour_lighting_contribution\": 1.0, \"entries\": [[0,0,0],[255,255,255],[255,127,0]], \"ranges\": [{\"start\": 0, \"end\": 1, \"is_primary_company_colour\": true}]}"

func TestGetPaletteFromJson(t *testing.T) {
	expectedRanges := []PaletteRange{{
		Start:                    0,
		End:                      1,
		IsPrimaryCompanyColour:   true,
		IsSecondaryCompanyColour: false,
		MaxGapInRegion:           6,
		ExpectedColourRange:      3,
	}}
	expectedEntries := []PaletteEntry{{R: 0, G: 0, B: 0}, {R: 255, G: 255, B: 255}, {R: 255, G: 127, B: 0}}

	palette, err := FromJson(strings.NewReader(exampleJson))

	if err != nil {
		t.Errorf("encountered error: %v", err)
	}

	if len(palette.Ranges) != len(expectedRanges) {
		t.Fatalf("ranges length %d is too short, expected %d", len(palette.Ranges), len(expectedRanges))
	}

	if len(palette.Entries) != len(expectedEntries) {
		t.Fatalf("entries length %d is too short, expected %d", len(palette.Entries), len(expectedEntries))
	}

	for i, r := range expectedRanges {
		if palette.Ranges[i] != r {
			t.Errorf("palette range %d not loaded correctly: was %v, expected %v", i, palette.Ranges[i], r)
		}
	}

	for i, e := range expectedEntries {
		if palette.Entries[i].R != e.R || palette.Entries[i].G != e.G || palette.Entries[i].B != e.B {
			t.Errorf("palette entry %d not loaded correctly: was %v, expected %v", i, palette.Entries[i], e)
		}
	}
}

func TestPalette_GetRGB(t *testing.T) {
	palette, _ := FromJson(strings.NewReader(exampleJson))
	palette.SetRanges([]PaletteRange{{Start: 2, End: 2, IsPrimaryCompanyColour: true}})

	expected := [][]uint16{{0, 0, 0}, {65535, 65535, 65535}, {38731, 38731, 38731}, {0, 0, 0}}

	for i, e := range expected {
		if rgb := palette.GetRGB(byte(i), true); uint16(rgb.R) != e[0] || uint16(rgb.G) != e[1] || uint16(rgb.B) != e[2] {
			t.Errorf("entry at %d not returned correctly: was [%v], expected %v", i, rgb, e)
		}
	}
}

func TestPalette_GetLitRGB(t *testing.T) {
	palette := Palette{Entries: []PaletteEntry{{R: 255, G: 0, B: 0}, {R: 64, G: 255, B: 128}}}

	testCases := []struct {
		index    byte
		lighting float64
		expected []uint16
	}{
		{0, 0, []uint16{65535, 0, 0}},
		{0, 1, []uint16{65535, 65535, 65535}},
		{0, -1, []uint16{0, 0, 0}},
		{0, 0.5, []uint16{65535, 32767, 32767}},
	}

	for _, testCase := range testCases {
		rgb := palette.GetLitRGB(testCase.index, testCase.lighting, 1.0, 0.0, 1.0, false, 1.0)
		result := []uint16{uint16(rgb.R), uint16(rgb.G), uint16(rgb.B)}
		if result[0] != testCase.expected[0] || result[1] != testCase.expected[1] || result[2] != testCase.expected[2] {
			t.Errorf("index %d with lighting %f returned %v, expected %v", testCase.index, testCase.lighting, result, testCase.expected)
		}
	}
}

func TestPalette_GetMaskColour(t *testing.T) {
	palette, _ := FromJson(strings.NewReader(exampleJson))
	palette.SetRanges([]PaletteRange{{Start: 2, End: 2, IsPrimaryCompanyColour: true}})

	expected := []byte{0, 0, 2}

	for i, e := range expected {
		if idx := palette.GetMaskColour(byte(i)); idx != e {
			t.Errorf("entry at %d not returned correctly: was %d, expected %d", i, idx, e)
		}
	}

}

func TestPalette_GetFromReader_DetectsDuplicateRanges(t *testing.T) {
	const json = "{\"entries\": [[0,0,0],[255,255,255],[255,127,0]], \"ranges\": [{\"start\": 0, \"end\": 1},{\"start\": 1, \"end\": 2}]}"
	_, err := FromJson(strings.NewReader(json))

	if err == nil || err.Error() != "range 1 overlaps colour 1" {
		t.Errorf("encountered unexpected error: %v", err)
	}
}

func TestPalette_GetGoPalette(t *testing.T) {
	expected := []color.RGBA{{0, 0, 0, 255}, {255, 255, 255, 255}, {255, 127, 0, 255}}

	palette, _ := FromJson(strings.NewReader(exampleJson))
	goPalette := palette.GetGoPalette()

	if len(goPalette) != len(expected) {
		t.Fatalf("palette lengths not equal - expected %d, got %d", len(expected), len(goPalette))
	}

	for i, e := range expected {
		if goPalette[i] != e {
			t.Errorf("palette entry %d does not match - expected %v, got %v", i, e, goPalette[i])
		}
	}
}

func TestPalette_GetLitIndexed(t *testing.T) {
	palette, _ := FromJson(strings.NewReader(exampleJson))
	palette.SetRanges([]PaletteRange{{Start: 0, End: 2}})

	testCases := []struct {
		index    byte
		lighting float64
		expected byte
	}{
		{1, 0, 1},
		{1, -1, 0},
		{1, 1, 2},
		{1, 5, 2},
	}

	for _, testCase := range testCases {
		if result := palette.GetLitIndexed(testCase.index, testCase.lighting); result != testCase.expected {
			t.Errorf("Get lit indexed colour for %d %f returned %d, expected %d", testCase.index, testCase.lighting, result, testCase.expected)
		}
	}

}
