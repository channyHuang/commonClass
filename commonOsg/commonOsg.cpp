#include "commonOsg.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

std::vector<std::string> splitString(std::string& str, char c) {
	std::vector<std::string> res;
	int stIdx = 0, endIdx = str.length() - 1;
	while (stIdx <= endIdx && str[stIdx] == ' ') stIdx++;
	while (endIdx >= stIdx && str[endIdx] == ' ') endIdx--;
	if (stIdx > endIdx) return res;
	str = str.substr(stIdx, endIdx - stIdx + 1);
	size_t stpos = 0;
	while (stpos != std::string::npos) {
		size_t nextpos = str.find_first_of(c, stpos);
		if (nextpos != std::string::npos) {
			res.push_back(str.substr(stpos, nextpos - stpos));

			stpos = nextpos + 1;
			while (stpos <= endIdx && str[stpos] == c) stpos++;
			if (stpos > endIdx) break;
		}
		else {
			res.push_back(str.substr(stpos));
			break;
		}
	}
	return res;
}

osg::Node* createOsgGlslNode(osg::ref_ptr<osg::Geometry> pGeom, osgViewer::Viewer* pViewer, std::string sTextrueName)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(osgDB::readRefShaderFile(osg::Shader::VERTEX, "E:/projects/work_svn/framework/osg/shaderfile/vshader_demo.glsl"));
	program->addShader(osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "E:/projects/work_svn/framework/osg/shaderfile/fshader_demo.glsl"));

	osg::ref_ptr<osg::StateSet> ss = pGeom->getOrCreateStateSet();
	ss->setAttribute(program);

	//// uniform
	osg::Uniform* mvpUniform = ss->getOrCreateUniform("mvpMat", osg::Uniform::FLOAT_MAT4);
	mvpUniform->setUpdateCallback(new MVPCallback(pViewer->getCamera()));
	osg::Uniform* viewUni = ss->getOrCreateUniform("viewMatInv", osg::Uniform::FLOAT_MAT4);
	viewUni->setUpdateCallback(new ViewCallback(pViewer->getCamera()));
	osg::Uniform* lightPosUni = ss->getOrCreateUniform("lightPos", osg::Uniform::FLOAT_VEC3);
	lightPosUni->setUpdateCallback(new LightPosCallback(pViewer->getCamera()));

	ss->setTextureAttributeAndModes(0, createTexture(sTextrueName));
	ss->addUniform(new osg::Uniform("textureMap", 0));

	// in 
	program->addBindAttribLocation("ivertex", 1);
	program->addBindAttribLocation("inormal", 2);
	program->addBindAttribLocation("itexcoord", 3);

	pGeom->setVertexAttribArray(1, pGeom->getVertexArray(), osg::Array::Binding::BIND_PER_VERTEX);
	if (pGeom->getNormalArray()) {
		pGeom->setVertexAttribArray(2, pGeom->getNormalArray(), osg::Array::Binding::BIND_PER_VERTEX);
	}
	else {
		osg::ref_ptr<osg::Vec3Array> normal = new osg::Vec3Array();
		normal->push_back(osg::Vec3(0, 1, 0));
		pGeom->setVertexAttribArray(2, normal, osg::Array::Binding::BIND_OVERALL);
	}
	pGeom->setVertexAttribArray(3, pGeom->getTexCoordArray(0), osg::Array::Binding::BIND_PER_VERTEX);

	return pGeom.release();
}

osg::ref_ptr<osg::Geometry> createGlslGeom(const tinyobj::attrib_t& attrib,
	const std::vector<tinyobj::shape_t>& shapes, osgViewer::Viewer* pViewer, const std::string& sTex)
{
	osg::ref_ptr<osg::Geometry> pGeom = new osg::Geometry;

	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(osgDB::readRefShaderFile(osg::Shader::VERTEX, "E:/projects/work_svn/framework/osg/shaderfile/vshader_demo.glsl"));
	program->addShader(osgDB::readRefShaderFile(osg::Shader::FRAGMENT, "E:/projects/work_svn/framework/osg/shaderfile/fshader_demo.glsl"));

	osg::ref_ptr<osg::StateSet> ss = pGeom->getOrCreateStateSet();
	ss->setAttribute(program);

	//// uniform
	osg::Uniform* mvpUniform = ss->getOrCreateUniform("mvpMat", osg::Uniform::FLOAT_MAT4);
	mvpUniform->setUpdateCallback(new MVPCallback(pViewer->getCamera()));
	osg::Uniform* viewUni = ss->getOrCreateUniform("viewMatInv", osg::Uniform::FLOAT_MAT4);
	viewUni->setUpdateCallback(new ViewCallback(pViewer->getCamera()));
	osg::Uniform* lightPosUni = ss->getOrCreateUniform("lightPos", osg::Uniform::FLOAT_VEC3);
	lightPosUni->setUpdateCallback(new LightPosCallback(pViewer->getCamera()));

	ss->setTextureAttributeAndModes(0, createTexture(sTex));
	ss->addUniform(new osg::Uniform("textureMap", 0));

	//======================
	osg::ref_ptr<osg::Vec3Array> vvertex = new osg::Vec3Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec2Array> vcoords = new osg::Vec2Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec3Array> vnormal = new osg::Vec3Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(attrib.vertices.size());

	bool bHasNormal = attrib.normals.size() > 0;
	bool bHasColor = attrib.colors.size() > 0;
	bool bHasCoords = attrib.texcoords.size() > 0;

#pragma omp parallel for num_threads(30)
	int idx = 0;
	for (int i = 0; i < attrib.vertices.size(); i += 3) {
		vvertex->at(idx) = (osg::Vec3(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]));
		if (bHasNormal) {
			vnormal->at(idx) = (osg::Vec3(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]));
		}
		else {
			vnormal->at(idx) = (osg::Vec3(0, 1, 0));
		}
		if (bHasColor) {
			vcolors->at(idx) = (osg::Vec4(attrib.colors[i], attrib.colors[i + 1], attrib.colors[i + 2], 1.f));
		}
		if (bHasCoords) {
			vcoords->at(idx) = (osg::Vec2(attrib.texcoords[(idx << 1)], attrib.texcoords[(idx << 1) + 1]));
		}
		idx++;
	}

#pragma omp parallel for
	for (int i = 0; i < shapes.size(); ++i) {
		std::vector<unsigned int> vindices;
		for (int j = 0; j < shapes[i].mesh.indices.size(); ++j) {
			auto vertexInfo = shapes[i].mesh.indices[j];
			if (vertexInfo.vertex_index == vertexInfo.texcoord_index) {
				vindices.push_back(vertexInfo.vertex_index);
			}
			else {
				vindices.push_back(vvertex->size());

				int vidx = vertexInfo.vertex_index * 3;
				vvertex->push_back(osg::Vec3(attrib.vertices[vidx], attrib.vertices[vidx + 1], attrib.vertices[vidx + 2]));
				if (bHasColor) {
					vcolors->push_back(osg::Vec4(attrib.colors[vidx], attrib.colors[vidx + 1], attrib.colors[vidx + 2], 1.f));
				}
				if (bHasNormal) {
					int ni = shapes[i].mesh.indices[j].normal_index * 3;
					vnormal->push_back(osg::Vec3(attrib.normals[ni], attrib.normals[ni + 1], attrib.normals[ni + 2]));
				}
				else {
					vnormal->push_back(osg::Vec3(0, 1, 0));
				}
				if (bHasCoords) {
					int ci = (shapes[i].mesh.indices[j].texcoord_index << 1);
					vcoords->push_back(osg::Vec2(attrib.texcoords[ci], attrib.texcoords[ci + 1]));
				}
			}
		}
		pGeom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, vindices.size(), vindices.data()));
	}

	pGeom->setVertexArray(vvertex);
	if (bHasNormal) pGeom->setNormalArray(vnormal, osg::Array::BIND_PER_VERTEX);
	if (bHasColor) pGeom->setColorArray(vcolors, osg::Array::BIND_PER_VERTEX);
	if (bHasCoords) pGeom->setTexCoordArray(0, vcoords);

	// in 
	program->addBindAttribLocation("ivertex", 1);
	program->addBindAttribLocation("inormal", 2);
	program->addBindAttribLocation("itexcoord", 3);

	pGeom->setVertexAttribArray(1, vvertex, osg::Array::Binding::BIND_PER_VERTEX);
	if (bHasNormal > 0) {
		pGeom->setVertexAttribArray(2, vnormal, osg::Array::Binding::BIND_PER_VERTEX);
	}
	else {
		pGeom->setVertexAttribArray(2, vnormal, osg::Array::Binding::BIND_OVERALL);
	}
	pGeom->setVertexAttribArray(3, vcoords, osg::Array::Binding::BIND_PER_VERTEX);

	return pGeom.release();
}

osg::ref_ptr<osg::Geometry> createGeom(const tinyobj::attrib_t& attrib, const std::vector<tinyobj::shape_t>& shapes) {
	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> vvertex = new osg::Vec3Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec2Array> vcoords = new osg::Vec2Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec3Array> vnormal = new osg::Vec3Array(attrib.vertices.size());
	osg::ref_ptr<osg::Vec4Array> vcolors = new osg::Vec4Array(attrib.vertices.size());

	bool bHasNormal = attrib.normals.size() > 0;
	bool bHasColor = attrib.colors.size() > 0;
	bool bHasCoords = attrib.texcoords.size() > 0;

#pragma omp parallel for num_threads(30)
	int idx = 0;
	for (int i = 0; i < attrib.vertices.size(); i += 3) {
		vvertex->at(idx) = (osg::Vec3(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]));
		if (bHasNormal) {
			vnormal->at(idx) = (osg::Vec3(attrib.normals[i], attrib.normals[i + 1], attrib.normals[i + 2]));
		}
		if (bHasColor) {
			vcolors->at(idx) = (osg::Vec4(attrib.colors[i], attrib.colors[i + 1], attrib.colors[i + 2], 1.f));
		}
		if (bHasCoords) {
			vcoords->at(idx) = (osg::Vec2(attrib.texcoords[(idx << 1)], attrib.texcoords[(idx << 1) + 1]));
		}
		idx++;
	}

#pragma omp parallel for
	for (int i = 0; i < shapes.size(); ++i) {
		std::vector<unsigned int> vindices;
		for (int j = 0; j < shapes[i].mesh.indices.size(); ++j) {
			auto vertexInfo = shapes[i].mesh.indices[j];
			if (vertexInfo.vertex_index == vertexInfo.texcoord_index) {
				vindices.push_back(vertexInfo.vertex_index);
			}
			else {
				vindices.push_back(vvertex->size());

				int vidx = vertexInfo.vertex_index * 3;
				vvertex->push_back(osg::Vec3(attrib.vertices[vidx], attrib.vertices[vidx + 1], attrib.vertices[vidx + 2]));
				if (bHasColor) {
					vcolors->push_back(osg::Vec4(attrib.colors[vidx], attrib.colors[vidx + 1], attrib.colors[vidx + 2], 1.f));
				}
				if (bHasNormal) {
					int ni = shapes[i].mesh.indices[j].normal_index * 3;
					vnormal->push_back(osg::Vec3(attrib.normals[ni], attrib.normals[ni + 1], attrib.normals[ni + 2]));
				}
				if (bHasCoords) {
					int ci = (shapes[i].mesh.indices[j].texcoord_index << 1);
					vcoords->push_back(osg::Vec2(attrib.texcoords[ci], attrib.texcoords[ci + 1]));
				}
			}
		}
		geom->addPrimitiveSet(new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, vindices.size(), vindices.data()));
	}

	geom->setVertexArray(vvertex);
	if (bHasNormal) geom->setNormalArray(vnormal, osg::Array::BIND_PER_VERTEX);
	if (bHasColor) geom->setColorArray(vcolors, osg::Array::BIND_PER_VERTEX);
	if (bHasCoords) geom->setTexCoordArray(0, vcoords);

	if (shapes.size() <= 0) {
		geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vvertex->size()));
	}

	return geom.release();
}

bool loadGeomUsingTinyobj(const char* filename, const char* basepath, osgViewer::Viewer* pViewer, bool bUseGlsl) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	bool triangulate = true;

	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
		filename, basepath, triangulate);

	if (!warn.empty()) {
		std::cout << "WARN: " << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << "ERR: " << err << std::endl;
	}

	if (!ret) {
		printf("Failed to load/parse .obj.\n");
		return false;
	}

	if (bUseGlsl) {
		auto pGeom = createGlslGeom(attrib, shapes, pViewer, basepath);

		pGeom->dirtyGLObjects();
		osg::Group* pGroup = dynamic_cast<osg::Group*>(pViewer->getSceneData());
		if (pGroup) {
			pGroup->addChild(pGeom);
		}
	}
	else {
		auto pGeom = createGeom(attrib, shapes);

		pGeom->dirtyGLObjects();
		osg::Group* pGroup = dynamic_cast<osg::Group*>(pViewer->getSceneData());
		if (pGroup) {
			pGroup->addChild(pGeom);
		}
	}

	return true;
}

osg::ref_ptr<osg::Geometry> createGlslGeomUsingTinyobj(const char* filename, const char* basepath, osgViewer::Viewer* pViewer)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename, basepath);

	if (!warn.empty()) {
		std::cout << "WARN: " << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << "ERR: " << err << std::endl;
	}

	if (!ret) {
		printf("Failed to load/parse .obj.\n");
		return nullptr;
	}

	return createGlslGeom(attrib, shapes, pViewer, basepath);
}

osg::ref_ptr<osg::Geometry> createGeomUsingTinyobj(const char* filename, const char* basepath) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename, basepath);

	if (!warn.empty()) {
		std::cout << "WARN: " << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << "ERR: " << err << std::endl;
	}

	if (!ret) {
		printf("Failed to load/parse .obj.\n");
		return nullptr;
	}

	return createGeom(attrib, shapes);
}

osg::ref_ptr<osg::Geometry> loadOsgNode(const std::string& sFileName) {
	size_t pos = sFileName.find_last_of('.');
	std::string ext = sFileName.substr(pos + 1);
	if (std::strcmp(ext.c_str(), "obj") == 0) {
	}

	// read model
	osg::ref_ptr<osg::Node> root = osgDB::readRefNodeFile(sFileName);
	if (!root.valid())
	{
		std::cout << __FUNCTION__ << " load file error : " << sFileName.c_str() << std::endl;
		return nullptr;
	}
	GeometryFinder finder;
	root->accept(finder);
	if (!finder._geom) {
		std::cout << __FUNCTION__ << " file error (not a geometry): " << sFileName.c_str() << std::endl;
		return nullptr;
	}
	return finder._geom;
}

osg::Node* createText(const osg::Vec3& position, float characterSize, const std::string& message, float minScale, float maxScale)
{
	std::string timesFont("fonts/arial.ttf");

	osgText::Text* text = new osgText::Text;
	text->setCharacterSize(characterSize);
	text->setText(message);
	text->setFont(timesFont);
	text->setAlignment(osgText::Text::CENTER_CENTER);

	osg::Geode* geode = new osg::Geode;
	geode->addDrawable(text);
	geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	osg::AutoTransform* at = new osg::AutoTransform;
	at->addChild(geode);

	at->setAutoRotateMode(osg::AutoTransform::ROTATE_TO_SCREEN);
	at->setAutoScaleToScreen(true);
	at->setMinimumScale(minScale);
	at->setMaximumScale(maxScale);
	at->setPosition(position);

	return at;
}

osg::ref_ptr<osg::LineWidth> getNewLineWidth(float width) {
	osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth();
	linewidth->setWidth(width);
	return linewidth;
}

osg::ref_ptr<osg::Geode> createAxis(float len)
{
	osg::Geode* geode(new osg::Geode());
	osg::Geometry* geometry(new osg::Geometry());

	osg::Vec3Array* vertices(new osg::Vec3Array());
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(len, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, len, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, 0.0));
	vertices->push_back(osg::Vec3(0.0, 0.0, len));
	geometry->setVertexArray(vertices);

	osg::Vec4Array* colors(new osg::Vec4Array());
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
	geometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
	geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));

	geode->addDrawable(geometry);

	osg::ref_ptr<osg::LineWidth> linewidth = getNewLineWidth(2.f);
	geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

	return geode;
}

osg::Group* createSunLight()
{
	osg::LightSource* sunLightSource = new osg::LightSource;

	osg::Light* sunLight = sunLightSource->getLight();
	sunLight->setLightNum(0);
	sunLight->setPosition(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	sunLight->setAmbient(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	//sunLight->setDiffuse(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));

	sunLightSource->setLight(sunLight);
	sunLightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	sunLightSource->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::LightModel* lightModel = new osg::LightModel;
	lightModel->setAmbientIntensity(osg::Vec4(0.0f, 0.0f, 0.0f, 1.0f));
	sunLightSource->getOrCreateStateSet()->setAttribute(lightModel);

	return sunLightSource;
}

osg::TextureCubeMap* readCubeMap()
{
	osg::TextureCubeMap* cubemap = new osg::TextureCubeMap;
#define CUBEMAP_FILENAME(face) "texture/" #face ".bmp"

	osg::ref_ptr<osg::Image>imagePosX = osgDB::readRefImageFile(CUBEMAP_FILENAME(posx));
	osg::ref_ptr<osg::Image>imageNegX = osgDB::readRefImageFile(CUBEMAP_FILENAME(negx));
	osg::ref_ptr<osg::Image>imagePosY = osgDB::readRefImageFile(CUBEMAP_FILENAME(posy));
	osg::ref_ptr<osg::Image>imageNegY = osgDB::readRefImageFile(CUBEMAP_FILENAME(negy));
	osg::ref_ptr<osg::Image>imagePosZ = osgDB::readRefImageFile(CUBEMAP_FILENAME(posz));
	osg::ref_ptr<osg::Image>imageNegZ = osgDB::readRefImageFile(CUBEMAP_FILENAME(negz));

	if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ)
	{
		cubemap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
		cubemap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
		cubemap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
		cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
		cubemap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
		cubemap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);

		cubemap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
		cubemap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
		cubemap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);

		cubemap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
		cubemap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	}

	return cubemap;
}

class MoveEarthySkyWithEyePointTransform : public osg::Transform
{
public:
	/** Get the transformation matrix which moves from local coords to world coords.*/
	virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
		if (cv)
		{
			osg::Vec3 eyePointLocal = cv->getEyeLocal();
			matrix.preMultTranslate(eyePointLocal);
		}
		return true;
	}

	/** Get the transformation matrix which moves from world coords to local coords.*/
	virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
		if (cv)
		{
			osg::Vec3 eyePointLocal = cv->getEyeLocal();
			matrix.postMultTranslate(-eyePointLocal);
		}
		return true;
	}
};

struct TexMatCallback : public osg::NodeCallback
{
public:

	TexMatCallback(osg::TexMat& tm) :
		_texMat(tm)
	{
	}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
		if (cv)
		{
			const osg::Matrix& MV = *(cv->getModelViewMatrix());
			const osg::Matrix R = osg::Matrix::rotate(osg::DegreesToRadians(112.0f), 0.0f, 0.0f, 1.0f) *
				osg::Matrix::rotate(osg::DegreesToRadians(90.0f), 1.0f, 0.0f, 0.0f);

			osg::Quat q = MV.getRotate();
			const osg::Matrix C = osg::Matrix::rotate(q.inverse());

			_texMat.setMatrix(C * R);
		}

		traverse(node, nv);
	}

	osg::TexMat& _texMat;
};

osg::Node* createSkyBox()
{
	osg::StateSet* stateset = new osg::StateSet();

	osg::TexEnv* te = new osg::TexEnv;
	te->setMode(osg::TexEnv::REPLACE);
	stateset->setTextureAttributeAndModes(0, te, osg::StateAttribute::ON);

	osg::TexGen* tg = new osg::TexGen;
	tg->setMode(osg::TexGen::NORMAL_MAP);
	stateset->setTextureAttributeAndModes(0, tg, osg::StateAttribute::ON);

	osg::TexMat* tm = new osg::TexMat;
	stateset->setTextureAttribute(0, tm);

	osg::TextureCubeMap* skymap = readCubeMap();
	stateset->setTextureAttributeAndModes(0, skymap, osg::StateAttribute::ON);

	stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
	stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);

	// clear the depth to the far plane.
	osg::Depth* depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setRange(1.0, 1.0);
	stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

	stateset->setRenderBinDetails(-1, "RenderBin");

	osg::Drawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 100));

	osg::Geode* geode = new osg::Geode;
	geode->setCullingActive(false);
	geode->setStateSet(stateset);
	geode->addDrawable(drawable);


	osg::Transform* transform = new MoveEarthySkyWithEyePointTransform;
	transform->setCullingActive(false);
	transform->addChild(geode);

	osg::ClearNode* clearNode = new osg::ClearNode;
	//  clearNode->setRequiresClear(false);
	clearNode->setCullCallback(new TexMatCallback(*tm));
	clearNode->addChild(transform);

	return clearNode;
}

osg::ref_ptr<osg::Program> loadShaderPrograms(const std::string& name, 
											  const std::string& vshaderName, const std::string& fshaderName) {
	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->setName(name);
	program->addShader(osgDB::readRefShaderFile(osg::Shader::VERTEX, vshaderName));
	program->addShader(osgDB::readRefShaderFile(osg::Shader::FRAGMENT, fshaderName));

	program->addBindAttribLocation("o_vertex", 1);
	program->addBindAttribLocation("o_normal", 2);

	return program;
}

osg::ref_ptr<osg::Program> loadShaderPrograms(osg::ref_ptr<osg::Geometry> pGeom, std::string vshader, std::string fshader, std::unordered_map<std::string, int>& inMap)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	program->addShader(osgDB::readRefShaderFile(osg::Shader::VERTEX, vshader));
	program->addShader(osgDB::readRefShaderFile(osg::Shader::FRAGMENT, fshader));

	osg::ref_ptr<osg::StateSet> ss = pGeom->getOrCreateStateSet();
	ss->setAttribute(program);

	// in 
	for (auto itr = inMap.begin(); itr != inMap.end(); itr++) {
		program->addBindAttribLocation(itr->first, itr->second);
	}
	return program;
}

osg::ref_ptr<osg::LightSource> createLight(const osg::Vec3& pos, int num, const osg::BoundingSphere& bs, osg::ref_ptr<osg::Group> root) {
	float modelSize = bs.radius();
	osg::ref_ptr<osg::LightSource> light = new osg::LightSource;
	light->getLight()->setLightNum(num);
	light->getLight()->setPosition(osg::Vec4(pos.x(), pos.y(), pos.z(), 1.f));
	light->getLight()->setAmbient(osg::Vec4(1.f, 1.f, 1.f, 1.f));
	light->getLight()->setDiffuse(osg::Vec4(1.f, 1.f, 1.f, 1.f));
	light->getLight()->setConstantAttenuation(1.0f);
	light->getLight()->setLinearAttenuation(2.0f / modelSize);
	light->getLight()->setQuadraticAttenuation(2.0f / osg::square(modelSize));

	light->setLocalStateSetModes(osg::StateAttribute::ON);
	light->setStateSetModes(*(root->getOrCreateStateSet()), osg::StateAttribute::ON);
	light->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

	osg::LightModel* lightModel = new osg::LightModel;
	lightModel->setAmbientIntensity(osg::Vec4(pos.x(), pos.y(), pos.z(), 1.0f));
	light->getOrCreateStateSet()->setAttribute(lightModel);
	
	return light;
}

osg::ref_ptr<osg::Material> createMaterial() {
	osg::ref_ptr<osg::Material> material = new osg::Material;
	material->setColorMode(osg::Material::AMBIENT_AND_DIFFUSE);
	material->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
	material->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(.5, .5, .5, 1));
	material->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
	return material;
}

osg::Texture2D* createTexture(const std::string& fileName)
{
	osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
	texture->setImage(osgDB::readRefImageFile(fileName));
	texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
	texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
	texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
	texture->setMaxAnisotropy(16.0f);
	return texture.release();
}

osg::Texture2D* createTexture(osg::Vec2s size = osg::Vec2s(1024, 1024)) {
	{ // Setup shadow texture
		osg::Texture2D* texture = new osg::Texture2D;
		texture->setTextureSize(size.x(), size.y());
		texture->setInternalFormat(GL_DEPTH_COMPONENT);
		texture->setShadowComparison(true);
		texture->setShadowTextureMode(osg::Texture2D::LUMINANCE);
		texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
		texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);

		// the shadow comparison should fail if object is outside the texture
		texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP_TO_BORDER);
		texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP_TO_BORDER);
		texture->setBorderColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
		return texture;
	}
}

GLuint createTexture(const unsigned char* data, int width, int height) {
	GLuint textureID;
	const int length = width * height * 4;
	GLubyte* pixels = new GLubyte[length];
	memcpy(pixels, data, length * sizeof(char));

	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	free(pixels);
	return textureID;
}

osg::TextureRectangle* createFloatTextureRectangle(int textureSize)
{
	osg::ref_ptr<osg::TextureRectangle> tex2D = new osg::TextureRectangle;
	tex2D->setTextureSize(textureSize, textureSize);
	tex2D->setInternalFormat(GL_RGBA16F_ARB);
	tex2D->setSourceFormat(GL_RGBA);
	tex2D->setSourceType(GL_FLOAT);
	return tex2D.release();
}

void setUniform() {
	//osg::Program* program = loadShaderPrograms();
	//ss->setAttributeAndModes(program, osg::StateAttribute::ON);

	//osg::Matrix projMatrix = osg::Matrix(1.0f, 0.0f, 1.0f, 0.0f,
	//	0.0f, 0.0f, -1.0f, 0.0f,
	//	0.0f, 1.0f, -1.0f, 0.0f,
	//	0.0f, 0.0f, 1.0f, 0.0f);
	//osg::Uniform* projMatrixUniform = new osg::Uniform("projMatrix", projMatrix);
	//osg::Uniform* lightPosUniform = new osg::Uniform("lightPos", osg::Vec3(0.0, 0.0f, 70.0f));
	//osg::Uniform* colorUniform = new osg::Uniform("color", osg::Vec3(1.0, 0.0, 1.0));
	//ss->addUniform(projMatrixUniform);
	//ss->addUniform(lightPosUniform);
	//ss->addUniform(colorUniform);
}

void setUniform(osg::ref_ptr<osg::Geometry> pGeom, osg::ref_ptr<osgViewer::Viewer> pviewer) {
	osg::ref_ptr<osg::StateSet> ss = pGeom->getOrCreateStateSet();
	//// uniform
	osg::Uniform* mvpUniform = ss->getOrCreateUniform("RotationMatrix", osg::Uniform::FLOAT_MAT4);
	mvpUniform->setUpdateCallback(new MVPCallback(pviewer->getCamera()));
	//osg::Uniform* viewUni = ss->getOrCreateUniform("viewMatInv", osg::Uniform::FLOAT_MAT4);
	//viewUni->setUpdateCallback(new ViewCallback(pviewer->getCamera()));
	//osg::Uniform* lightPosUni = ss->getOrCreateUniform("lightPos", osg::Uniform::FLOAT_VEC3);
	//lightPosUni->setUpdateCallback(new LightPosCallback(pviewer->getCamera()));

	//ss->setTextureAttributeAndModes(0, createTexture("d:/osgfile/obj/textured_output.bmp"));
	//ss->addUniform(new osg::Uniform("textureMap", 0));
}

void setTexture(const std::string& name, osg::StateSet* stateset) {
	osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(name);
	if (image) {
		osg::Texture2D* texture = new osg::Texture2D;
		texture->setImage(image);
		stateset->setTextureAttributeAndModes(0, texture, osg::StateAttribute::ON);

		//osg::TexGen* texgen = new osg::TexGen;
		//texgen->setMode(osg::TexGen::SPHERE_MAP);
		//stateset->setTextureAttributeAndModes(0, texgen, osg::StateAttribute::ON);
	}
}

void setWireFrame(osg::StateSet* stateset, ShowType type) {
	osg::PolygonMode* polymode = new osg::PolygonMode;
	if (type == ShowType::SHOW_WIREFRAME) {
		polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
	}
	else if (type == ShowType::SHOW_FRONT) {
		polymode->setMode(osg::PolygonMode::FRONT, osg::PolygonMode::LINE);
		polymode->setMode(osg::PolygonMode::BACK, osg::PolygonMode::FILL);
	} else {
		polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::FILL);
	}
	stateset->setAttributeAndModes(polymode, osg::StateAttribute::ON);
	stateset->setAttributeAndModes(getNewLineWidth().get(), osg::StateAttribute::ON);
}

void clearSingleGroup(osg::ref_ptr<osg::Group>& node) {
	node->removeChildren(0, node->getNumChildren());
}

void setTexCombine(osg::StateSet* stateset) {
	osg::TexEnvCombine* te = new osg::TexEnvCombine;
	te->setCombine_RGB(osg::TexEnvCombine::ADD);
	te->setSource0_RGB(osg::TexEnvCombine::TEXTURE);
	te->setOperand0_RGB(osg::TexEnvCombine::SRC_COLOR);
	//te->setSource1_RGB(osg::TexEnvCombine::PRIMARY_COLOR);
	//te->setOperand1_RGB(osg::TexEnvCombine::SRC_COLOR);
	stateset->setTextureAttributeAndModes(0, te);
}

osg::Geometry* createPlanet(double radius, const osg::Vec4& color, const osg::Vec3& pos)
{
	// create a container that makes the sphere drawable
	osg::Geometry* sPlanetSphere = new osg::Geometry();

	{
		// set the single colour so bind overall
		osg::Vec4Array* colours = new osg::Vec4Array(1);
		(*colours)[0] = color;
		sPlanetSphere->setColorArray(colours, osg::Array::BIND_OVERALL);


		// now set up the coords, normals and texcoords for geometry
		unsigned int numX = 100;
		unsigned int numY = 50;
		unsigned int numVertices = numX * numY;

		osg::Vec3Array* coords = new osg::Vec3Array(numVertices);
		sPlanetSphere->setVertexArray(coords);

		osg::Vec3Array* normals = new osg::Vec3Array(numVertices);
		sPlanetSphere->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);

		osg::Vec2Array* texcoords = new osg::Vec2Array(numVertices);
		sPlanetSphere->setTexCoordArray(0, texcoords);
		sPlanetSphere->setTexCoordArray(1, texcoords);

		double delta_elevation = osg::PI / (double)(numY - 1);
		double delta_azim = 2.0 * osg::PI / (double)(numX - 1);
		float delta_tx = 1.0 / (float)(numX - 1);
		float delta_ty = 1.0 / (float)(numY - 1);

		double elevation = -osg::PI * 0.5;
		float ty = 0.0;
		unsigned int vert = 0;
		unsigned j;
		for (j = 0;
			j < numY;
			++j, elevation += delta_elevation, ty += delta_ty)
		{
			double azim = 0.0;
			float tx = 0.0;
			for (unsigned int i = 0;
				i < numX;
				++i, ++vert, azim += delta_azim, tx += delta_tx)
			{
				osg::Vec3 direction(cos(azim) * cos(elevation), sin(azim) * cos(elevation), sin(elevation));
				(*coords)[vert].set(direction * radius + pos);
				(*normals)[vert].set(direction);
				(*texcoords)[vert].set(tx, ty);
			}
		}

		for (j = 0;
			j < numY - 1;
			++j)
		{
			unsigned int curr_row = j * numX;
			unsigned int next_row = curr_row + numX;
			osg::DrawElementsUShort* elements = new osg::DrawElementsUShort(GL_QUAD_STRIP);
			for (unsigned int i = 0;
				i < numX;
				++i)
			{
				elements->push_back(next_row + i);
				elements->push_back(curr_row + i);
			}
			sPlanetSphere->addPrimitiveSet(elements);
		}
	}

	return sPlanetSphere;
	// set the object color
	//sPlanetSphere->setColor( color );

	// create a geode object to as a container for our drawable sphere object
	//osg::Geode* geodePlanet = new osg::Geode();

	// add our drawable sphere to the geode container
	//geodePlanet->addDrawable(sPlanetSphere);

	//return(geodePlanet);

}// end SolarSystem::createPlanet

//           |   i     j     k   | 
// S = 1 / 2 | bx-ax by-ay bz-az | = 1/2 |AB x AC|
//	         | cx-ax cy-ay cz-az |
float area(float ax, float ay, float az, float bx, float by, float bz, float cx, float cy, float cz) {
	float resx = (by - ay) * (cz - az) - (cy - ay) * (bz - az);
	float resy = (bx - ax) * (cz - az) - (cx - ax) * (bz - az);
	float resz = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay);
	float res = std::sqrt(resx * resx + resy * resy + resz * resz) * 0.5;
	return std::fabs(res);
}

float calcObjProjectArea(std::string sMeshName, std::string sPath) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	bool triangulate = true;

	std::string warn;
	std::string err;
	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
	    sMeshName.c_str(), sPath.c_str(), triangulate);

	if (!warn.empty()) {
	    std::cout << "WARN: " << warn << std::endl;
	}

	if (!err.empty()) {
	    std::cerr << "ERR: " << err << std::endl;
	}

	if (!ret) {
	    printf("Failed to load/parse .obj.\n");
	}
	float res = 0;
	for (const auto& shape : shapes) {
	    int idx = 0;
		std::vector<std::vector<float>> pts;
	    for (const auto& index : shape.mesh.indices) {
			std::vector<float> pt;
			pt.push_back(attrib.vertices[3 * index.vertex_index + 0]);
			pt.push_back(attrib.vertices[3 * index.vertex_index + 1]);
			pt.push_back(attrib.vertices[3 * index.vertex_index + 2]);
			pts.push_back(pt);

			if (pts.size() >= 3) {
				res += area(pts[0][0], pts[0][1], pts[0][2], pts[1][0], pts[1][1], pts[1][2], pts[2][0], pts[2][1], pts[2][2]);
				pts.clear();
			}
	    }
	}
	return res;
}

