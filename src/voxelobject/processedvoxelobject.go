package voxelobject

import (
	"colour"
	"geometry"
	"sync"
)

type ProcessedElement struct {
	Normal         geometry.Vector3
	AveragedNormal geometry.Vector3
	Occlusion      int
	Index          byte
	IsSurface      bool
}

type ProcessedVoxelObject struct {
	Elements         [][][]ProcessedElement
	borderedElements [][][]byte
	Size             geometry.Point
	Palette          *colour.Palette
}

const normalRadius = 3
const normalAverageDistance = 1
const occlusionRadius = 4
const accessBorder = 8

func (r RawVoxelObject) GetProcessedVoxelObject(pal *colour.Palette) (p ProcessedVoxelObject) {
	p.Size = r.Size()
	p.Palette = pal

	p.setElements(r)
	p.calculatePass(processFirstPassElement)
	p.calculatePass(processSecondPassElement)

	return
}

func (p *ProcessedVoxelObject) calculatePass(processor func(*ProcessedVoxelObject, int, int, int)) {
	wg := sync.WaitGroup{}
	wg.Add(p.Size.X)

	for x := 0; x < p.Size.X; x++ {
		thisX := x
		go func() {
			for y := 0; y < p.Size.Y; y++ {
				for z := 0; z < p.Size.Z; z++ {
					processor(p, thisX, y, z)
				}
			}
			wg.Done()
		}()
	}

	wg.Wait()

}

func processFirstPassElement(p *ProcessedVoxelObject, x int, y int, z int) {
	p.Elements[x][y][z].IsSurface = p.isSurface(x, y, z)
	p.Elements[x][y][z].Normal = p.calculateNormal(x, y, z)
}

func processSecondPassElement(p *ProcessedVoxelObject, x int, y int, z int) {
	p.Elements[x][y][z].AveragedNormal = p.getAverageNormal(x, y, z)
	p.Elements[x][y][z].Occlusion = p.getOcclusion(x, y, z)
}

func (p *ProcessedVoxelObject) getNormalRadius(index byte) (radius int) {
	radius = normalRadius + (p.Palette.GetSmoothness(index) * 2)
	if radius < 1 {
		return 1
	}
	return
}

func (p *ProcessedVoxelObject) calculateNormal(x, y, z int) (normal geometry.Vector3) {
	if !p.Elements[x][y][z].IsSurface {
		return
	}

	radius := p.getNormalRadius(p.Elements[x][y][z].Index)
	x += accessBorder
	y += accessBorder
	z += accessBorder

	for i := -radius; i <= radius; i++ {
		for j := -radius; j <= radius; j++ {
			for k := -radius; k <= radius; k++ {
				if (i*i)+(j*j)+(k*k) <= (radius*radius) && p.borderedElements[x+i][y+j][z+k] == 0 {
					normal = normal.Subtract(geometry.Vector3{X: float64(i), Y: float64(j), Z: float64(k)})
				}
			}
		}
	}

	if normal.Length() > 0.01 {
		return normal.Normalise()
	}

	return normal
}

func (p *ProcessedVoxelObject) getSafeDistance(x int, y int, z int, radius int) (int, int, int, int, int, int) {
	minI, maxI := -radius, radius
	if (x + minI) < 0 {
		minI -= x + minI
	}
	if (x + maxI) >= p.Size.X-1 {
		maxI -= (x + maxI) - (p.Size.X - 1)
	}

	minJ, maxJ := -radius, radius
	if (y + minJ) < 0 {
		minJ -= y + minJ
	}
	if (y + maxJ) >= p.Size.Y-1 {
		maxJ -= (y + maxJ) - (p.Size.Y - 1)
	}

	minK, maxK := -radius, radius
	if (z + minK) < 0 {
		minK -= z + minK
	}
	if (z + maxK) >= p.Size.Z-1 {
		maxK -= (z + maxK) - (p.Size.Z - 1)
	}
	return minI, maxI, minJ, maxJ, minK, maxK
}

func (p *ProcessedVoxelObject) getNormalAverageDistance(index byte) (distance int) {
	distance = normalAverageDistance + (p.Palette.GetSmoothness(index))
	if distance < 0 {
		return 0
	}
	return
}

func (p *ProcessedVoxelObject) getAverageNormal(x, y, z int) (normal geometry.Vector3) {
	if !p.Elements[x][y][z].IsSurface {
		return
	}

	smoothness := p.Palette.GetSmoothness(p.SafeGetData(x, y, z).Index)

	distance := p.getNormalAverageDistance(p.SafeGetData(x, y, z).Index)
	minI, maxI, minJ, maxJ, minK, maxK := p.getSafeDistance(x, y, z, distance)

	for i := minI; i <= maxI; i++ {
		for j := minJ; j <= maxJ; j++ {
			for k := minK; k <= maxK; k++ {
				if p.Elements[x+i][y+j][z+k].Index != 0 {
					if p.Palette.GetSmoothness(p.Elements[x+i][y+j][z+k].Index) == smoothness {
						normal = normal.Add(p.Elements[x+i][y+j][z+k].Normal)
					}
				}
			}
		}
	}

	if normal.Length() < 0.01 {
		return p.Elements[x][y][z].Normal
	}

	return normal.Normalise()
}

func (p *ProcessedVoxelObject) getOcclusion(x, y, z int) (occlusion int) {
	if !p.Elements[x][y][z].IsSurface {
		return
	}

	normal := p.Elements[x][y][z].AveragedNormal
	n := geometry.Vector3{X: float64(x), Y: float64(y), Z: float64(z)}.Subtract(normal.MultiplyByConstant(2.0))
	q, w, e := int(n.X), int(n.Y), int(n.Z)

	distance := occlusionRadius
	distanceF := float64(distance)

	minI, maxI, minJ, maxJ, minK, maxK := p.getSafeDistance(q, w, e, distance)

	for i := minI; i <= maxI; i++ {
		for j := minJ; j <= maxJ; j++ {
			for k := minK; k <= maxK; k++ {
				vec := geometry.Vector3{X: float64(i), Y: float64(j), Z: float64(k)}

				if vec.Length() < distanceF && vec.Dot(normal) < 0 {
					if p.Elements[q+i][w+j][e+k].IsSurface {
						occlusion++
						if occlusion >= 10 {
							return
						}
					}
				}
			}
		}
	}

	return
}

func (p *ProcessedVoxelObject) isSurface(x, y, z int) bool {
	// A voxel is a surface voxel if any of the adjacent directions is zero
	// The edges of the voxel object are trivially surface voxels
	return p.Elements[x][y][z].Index != 0 && (x == 0 || y == 0 || z == 0 ||
		x == p.Size.X-1 || y == p.Size.Y-1 || z == p.Size.Z-1 ||
		p.Elements[x+1][y][z].Index == 0 || p.Elements[x-1][y][z].Index == 0 ||
		p.Elements[x][y+1][z].Index == 0 || p.Elements[x][y-1][z].Index == 0 ||
		p.Elements[x][y][z-1].Index == 0 || p.Elements[x-1][y][z+1].Index == 0 ||
		p.Elements[x][y][z+1].Index == 0 || p.Elements[x+1][y][z+1].Index == 0 ||
		p.Elements[x][y-1][z+1].Index == 0 || p.Elements[x][y+1][z+1].Index == 0)
}

func (p *ProcessedVoxelObject) setElements(r RawVoxelObject) {
	p.Elements = make([][][]ProcessedElement, p.Size.X)
	p.borderedElements = make([][][]byte, p.Size.X+(accessBorder*2))

	for x := 0; x < p.Size.X+(accessBorder*2); x++ {
		p.borderedElements[x] = make([][]byte, p.Size.Y+(accessBorder*2))
		for y := 0; y < p.Size.Y+(accessBorder*2); y++ {
			p.borderedElements[x][y] = make([]byte, p.Size.Z+(accessBorder*2))
		}
	}

	for x := 0; x < p.Size.X; x++ {
		p.Elements[x] = make([][]ProcessedElement, p.Size.Y)
		for y := 0; y < p.Size.Y; y++ {
			p.Elements[x][y] = make([]ProcessedElement, p.Size.Z)
			for z := 0; z < p.Size.Z; z++ {
				p.Elements[x][y][z].Index = r[x][y][z]
				p.borderedElements[x+accessBorder][y+accessBorder][z+accessBorder] = r[x][y][z]
			}
		}
	}

}

func (pv *ProcessedVoxelObject) SafeGetData(x, y, z int) (pe ProcessedElement) {
	if x >= 0 && y >= 0 && z >= 0 && x < pv.Size.X && y < pv.Size.Y && z < pv.Size.Z {
		pe = pv.Elements[x][y][z]
	}

	return
}

func (pv *ProcessedVoxelObject) Invalid() bool {
	return pv.Size.X == 0 || pv.Size.Y == 0 || pv.Size.Z == 0
}
