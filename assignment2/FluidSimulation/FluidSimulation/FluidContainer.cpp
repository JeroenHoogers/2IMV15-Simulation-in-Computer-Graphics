#include "FluidContainer.h"
#include <algorithm>
#include <time.h>

using namespace std;

//-----------------------------------------------------------------------------------------------------------------//
FluidContainer::FluidContainer(float radius, float sceneWidth, float sceneHeight)
{
	_cellSize = radius;
	_radius = radius;

	_sceneWidth = sceneWidth;
	_sceneHeight = sceneHeight;

	m_GridRows = floorf(sceneHeight / _cellSize);
	m_GridCols = floorf(sceneWidth / _cellSize);

	_gridCounters = new int[m_GridRows * m_GridCols];
	//m_GridColors = new float[m_GridRows];
	_gridCells = new vector<int>[m_GridRows * m_GridCols];
	m_Neighbours = new vector<int>[m_GridRows * m_GridCols];

	m_CenterPoints = new Vec2f[m_GridRows * m_GridCols];

	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		//_gridCounters[i] = new int[m_GridCols];
		//m_GridColors[i] = new float[m_GridCols];
		//_gridCells[i] = new vector<int>[m_GridCols];

		//m_CenterPoints[i] = new Vec2f[m_GridCols];
		_gridCounters[i] = 0;
		_gridCells[i] = vector<int>();
		m_Neighbours[i] = vector<int>(90, -1);


		//for (int j = 0; j < m_GridCols; j++)
		//{
		//	_gridCounters[i][j] = 0;
		//	m_GridColors[i][j] = 0;
		//	//_gridCells[i][j] = vector<int>();

		// calculate centerpoints
		m_CenterPoints[i] = Vec2f(
			((i % m_GridCols) * _cellSize) + (_cellSize / 2) - 1,
			(int(i / m_GridCols) * _cellSize) + (_cellSize / 2) - 1);

		//}
	}


	// Initialize fluid renderer
	_fbo = 0;
	_fluidTexture = 0;
	_particleTexture = 0;
	_textureSize = 512;
	_threshold = 0.8f;
}

//-----------------------------------------------------------------------------------------------------------------//
FluidContainer::~FluidContainer()
{
	//// Release memory
	//for (int i = 0; i < _gridRows; i++)
	//{
	//	//for (int j = 0; j < _gridSize; j++)
	//	//{
	//	//	delete[] _gridCells[i][j];
	//	//}

	//	delete[] _gridCounters[i];
	//	delete[] _gridCells[i];
	//	delete[] _gridColors[i];
	//	delete[] _centerPoints[i];
	//}

	//delete[] _gridCounters;
	//delete[] _gridCells;
	//delete[] _gridColors;
	//delete[] _centerPoints;
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::UpdateGrid(vector<Particle*> particles)
{
	//ClearGrid();
	//time_t begin_time = clock();
	for (int i = 0; i < particles.size(); i++)
	{
		int id = CalcHash(particles[i]->m_Position);
		
		// Check if this particle moved to another cell since last timestep
		if (id != particles[i]->m_GridId)
		{
			if (particles[i]->m_GridId != -1)
			{
				// Remove from old cell
				_gridCells[particles[i]->m_GridId].erase(
					remove(_gridCells[particles[i]->m_GridId].begin(), _gridCells[particles[i]->m_GridId].end(), i), _gridCells[particles[i]->m_GridId].end()
					);
				_gridCounters[particles[i]->m_GridId]--;
			}

			particles[i]->m_GridId = id;

			if (!(id < 0 || id >= m_GridRows * m_GridCols))
			{
				// Add to new cell
				_gridCounters[id]++;
				_gridCells[id].push_back(i);
			}
			else
			{
				// Particle went off grid
				particles[i]->m_GridId = -1;
			}
		}
	}
	
	//time_t begin_time2 = clock();
	// Find neighbours for every cell w/ particle count > 0
	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		if (_gridCounters[i] > 0)
		{
			FindNeighbours(i);
		}
	}
	//std::cout << float(clock() - begin_time2) / CLOCKS_PER_SEC << std::endl;
}

//-----------------------------------------------------------------------------------------------------------------//
int FluidContainer::CalcHash(Vec2f position)
{
	return (
		int((position[0] / _cellSize) + (m_GridCols / 2.0f)) +					// x
		int((position[1] / _cellSize) + (m_GridRows / 2.0f)) * m_GridCols		// y
	);
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::FindNeighbours(int id)
{
	m_Neighbours[id].clear();
	//int id = CalcHash(position);
	//float width = _sceneWidth / _cellSize;

	//int x = 0;
	//int y = 0;

	// move to left cell
	int nid = id - (m_GridCols + 1);
	for (int i = 0; i < 9; i++)
	{
		if (i != 0)
		{
			if (i % 3 == 0)
				nid += (m_GridCols - 2); // move y value and go back 2 places
			else
				nid++;
		}

		// Check if this cell is within the grid
		if (!(nid < 0 || nid >= m_GridRows * m_GridCols))
		{
			// Add particles
			for (int j = 0; j < _gridCells[nid].size(); j++)
			{
				m_Neighbours[id].push_back(_gridCells[nid].at(j));
				//ids.push_back(_gridCells[id].at(j));
			}
			//m_Neighbours[id].insert(m_Neighbours[id].end(), _gridCells[nid].begin(), _gridCells[nid].end());
		}
	}
	//return m_Neighbours[id];
	// return ids;
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::setColor(Vec2f position, float color)
{
	//float width = _sceneWidth / _cellSize;
	int x = floorf((position[0] / _cellSize) + (m_GridCols / 2.0f));
	int y = floorf((position[1] / _cellSize) + (m_GridRows / 2.0f));

	// Check if this cell is within the grid
	if (!(x < 0 || y < 0 || x >= m_GridCols || y >= m_GridCols))
	{
		//m_GridColors[x][y] = color;
	}
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::ClearGrid()
{
	for (int i = 0; i < m_GridRows * m_GridCols; i++)
	{
		if (_gridCounters[i] != 0)
		{
			//_gridCounters[i] = 0;
			//_gridCells[i] = vector<int>();
			m_Neighbours[i] = vector<int>();
		}
	}
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::SetupFluidRendering()
{
	cout << "OpenGL Version: " << glGetString(GL_VERSION) << endl;

	// Initialize fluid shader
	//_fluidShader = Shader(
	//	"Content//Shaders//fluid.vert",
	//	"Content//Shaders//fluid.frag");

	// Generate fluid texture
	glGenTextures(1, &_fluidTexture);
	glBindTexture(GL_TEXTURE_2D, _fluidTexture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _textureSize, _textureSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

	// Generate FBO
	_fbo = 0;
	if (glutExtensionSupported("GL_EXT_framebuffer_object"))//GL_EXT_framebuffer_object))
	{
		glGenFramebuffers(1, &_fbo);
		glBindFramebuffer(GL_FRAMEBUFFER_EXT, _fbo);
		glFramebufferTexture2D(GL_FRAMEBUFFER_EXT, GL_COLOR_ATTACHMENT0_EXT, GL_TEXTURE_2D, _fluidTexture, 0);
		int result = glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);
		if (result != GL_FRAMEBUFFER_COMPLETE_EXT)
			cout << "Cannot initialize FBO" << endl;
	}

	if (_fbo != 0)
		glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);


	// Generate attenuation texture
	glGenTextures(1, &_particleTexture);
	glBindTexture(GL_TEXTURE_2D, _particleTexture);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	//if (!glutExtensionSupported("GL_ARB_texture_float"))
	//{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		float radius = 8.0f;
		int size = (radius * radius) * 2;
		float* imageData = GenerateParticleTexture(size);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_ALPHA32F_ARB, size, size, 0, GL_ALPHA, GL_FLOAT, imageData);
	//}
//	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _textureSize, _textureSize, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);


}

//-----------------------------------------------------------------------------------------------------------------//
float* FluidContainer::GenerateParticleTexture(int textureSize)
{
	float energy = 1.0f;
	float falloff = 0.3f;
	float energyThreshold = 0.8f;

	//new BlobRenderer(1.0f, 0.4f, 0.8f, BLOB_RADIUS, TEX_SIZE, Color.LightSkyBlue);
	
	// Convert particle size to the nearest 2^n
	double exponent = log2((double)textureSize);
	textureSize = (int)pow(2.0, exponent);
	//textureSize = 32;

	// Create Particle texture
	float* image = new float[textureSize * textureSize];
	int center = textureSize / 2;

	double centerHalf2 = (center / 2.0) * falloff;
	centerHalf2 = centerHalf2 * centerHalf2;
	float maxThreshold = energyThreshold - (energyThreshold * 0.11f);
	for (int x = 0; x < textureSize; x++)
	{
		for (int y = 0; y < textureSize; y++)
		{
			Vec2f dist = Vec2f(x - center, y - center);
			// calculate energy of this point using gaussian
			float dist2 = dist * dist;
			float en = (float)exp((double)-dist2 / centerHalf2) * energy;

			// clamp 
			if (en < 0.0f)
				en = 0.0f;
			else if (en > maxThreshold)
				en = maxThreshold;

			// set pixel alpha
			image[x * textureSize + y] = en;
		}
	}
	return image;
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::RenderFluidToTexture(vector<Particle*> particles, float radius)
{
	
	// Clear if the fbo already exists
	if (_fbo != 0)
	{
		glBindFramebuffer(GL_FRAMEBUFFER_EXT, _fbo);
		glClear(GL_COLOR_BUFFER_BIT);
	}

	glPushAttrib(GL_VIEWPORT_BIT);
	glViewport(0, 0, _textureSize, _textureSize);

	glDisable(GL_ALPHA_TEST);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _particleTexture);

	// Transform radius
	float ratio = (radius * radius) / _textureSize;
	float radiusW = ratio * 2.0f;
	float radiusH = ratio * 2.0f;

	// Draw particles
	glBegin(GL_QUADS);
	for (int i = 0; i < particles.size(); i++)
	{
		if (!particles[i]->m_isBoundary)
		{
			Vec2f pos = particles[i]->m_Position;
			glTexCoord2f(0.0f, 0.0f);
			glVertex2f(pos[0] - radiusW, pos[1] - radiusH);
			glTexCoord2f(1.0f, 0.0f);
			glVertex2f(pos[0] + radiusW, pos[1] - radiusH);
			glTexCoord2f(1.0f, 1.0f);
			glVertex2f(pos[0] + radiusW, pos[1] + radiusH);
			glTexCoord2f(0.0f, 1.0f);
			glVertex2f(pos[0] - radiusW, pos[1] + radiusH);
		}
	}
	glEnd();

	// Copy framebuffer to texture
	if (_fbo == 0)
	{
		glBindTexture(GL_TEXTURE_2D, _fluidTexture);
		glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, _textureSize, _textureSize, 0);
	}
	else
	{
		glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
	}

	glPopAttrib();
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::RenderFluid(vector<Particle*> particles, float radius)
{

	// Setup rendering
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_TEXTURE_BIT);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glAlphaFunc(GL_GEQUAL, 0.8f);

	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

	// Render fluid to texture
	RenderFluidToTexture(particles, radius);

	// Thresholding
	glEnable(GL_ALPHA_TEST);

	glDisable(GL_TEXTURE_2D);
	glColor4f(0.4f, 0.4f, 1.0f, 1.0f);

	glBegin(GL_QUADS);
		glVertex2f(-1.0f, -1.0f);
		glVertex2f(1.0f, -1.0f);
		glVertex2f(1.0f, 1.0f);
		glVertex2f(-1.0f, 1.0f);
	glEnd();


	//_fluidShader.bind();
	// Render texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, _fluidTexture);


	glBegin(GL_QUADS);
		glTexCoord2f(0.0f, 0.0f);
		glVertex2f(-1.0f, -1.0f);

		glTexCoord2f(1.0f, 0.0f);
		glVertex2f(1.0f, -1.0f);

		glTexCoord2f(1.0f, 1.0f);
		glVertex2f(1.0f, 1.0f);

		glTexCoord2f(0.0f, 1.0f);
		glVertex2f(-1.0f, 1.0f);
	glEnd();
	//_fluidShader.unbind();

	// End rendering
	glPopAttrib();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glDisable(GL_ALPHA_TEST);
}

//-----------------------------------------------------------------------------------------------------------------//
void FluidContainer::draw()
{
	// Draw boundaries
	glLineWidth(4);
	glColor3f(0.3, 0.3, 0.3);
	glBegin(GL_LINE_LOOP);

	glVertex2f(-0.95f, -0.95f);
	glVertex2f(-0.95f, 0.94f);
	glVertex2f(0.94f, 0.94f);
	glVertex2f(0.94f, -0.95f);

	glEnd();
	glLineWidth(1);

	//glColor4f(1, 1, 1, 0.1);

	////glLineWidth(5.0);
	//glPointSize(6.0);


	////glTranslatef(-1, -1, 0);
	//for (int i = 0; i < m_GridRows * m_GridCols; i++)
	//{
	//	//glBegin(GL_LINES);
	//	//glVertex2f(-1, i * _cellSize - 1);
	//	//glVertex2f(_sceneWidth - 1, i * _cellSize - 1);
	//	//glEnd();

	//	glBegin(GL_POINTS);

	//	//if(_gridCounters[i] > 0)
	//	//{ 
	//	glColor4f(1, 1, 1, 0.1f + (_gridCounters[i] ));
	//			
	//	glVertex2f(m_CenterPoints[i][0], m_CenterPoints[i][1]);
	//	//}
	//	glEnd();
	//}
	//glBegin(GL_LINES);
	//glColor4f(1, 1, 1, 0.1);
	////for (int j = 0; j < m_GridCols; j++)
	////{
	////	glVertex2f(j * _cellSize - 1, -1);
	////	glVertex2f(j * _cellSize - 1, _sceneHeight);
	////}

	////glTranslatef(2, 2, 0);
	//glEnd();
}
