// TestUltraMesh.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <chrono>
#include "../UltraMesh.h"
#include "../window.h"
#include "pg/pgrender.h"
#include "C:/Eigen3.3.7/Eigen/Dense"


#define PROMPT(_str) PgWindowText(_str);
#define JOURNAL_DEBUG 0
#define MINIMAL_WALL_THICKNESS 6
#define REMESH true
#define TIME_INTERVAL(end, start) double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / 1000.0


PTSolid ReadSTL(PTEnvironment env, std::wstring fileName)
{
	PTStream stream = PV_ENTITY_NULL;
	PTStatus status = PV_STATUS_OK;

	std::string sFileName = std::string(fileName.cbegin(), fileName.cend());
	status += PFStreamFileOpen(PTString(sFileName.c_str()), PV_FILE_READ, NULL, &stream);

	PTSolidReadOpts solidReadOptions;
	PMInitSolidReadOpts(&solidReadOptions);
	solidReadOptions.file_colour_format = PV_FILE_COLOUR_STL_RGB;

	PTSolid model = PV_ENTITY_NULL;
	status += PFSolidRead(env, PV_SOLID_DATA_STL, stream, &solidReadOptions, &model);
	PFStreamClose(stream);
	return model;
}

PTSolid UltraMesh2PTSolid(UltraMesh& mesh, PTEnvironment env)
{
	PTSolid newSolid = PV_ENTITY_NULL;
	PTStatus status = PV_STATUS_OK;
	PTBeginSolidOpts BeginSolidOptions;
	PMInitBeginSolidOpts(&BeginSolidOptions);
	double Triangle[3][3];
	PTPointer Trig;

	Trig = &Triangle;
	status = PFSolidCreateBegin(env, PV_SOLID_CREATE_TRIANGLES, &BeginSolidOptions, &newSolid);
	for (auto face : mesh.m_faces)
	{
		Eigen::Vector3i idx = face.Vertices();
		for (int i = 0; i < 3; i++)
		{
			Eigen::Vector3d position = mesh.m_vertices[idx[i]].Position();
			memcpy(&Triangle[i][0], &position, sizeof(PTPoint));
		}
		status += PFSolidCreateTriangle(newSolid, Trig, NULL);
	}
	status += PFSolidCreateEnd(newSolid, NULL);
	if (status != PV_STATUS_OK)
		newSolid = 0;
	return newSolid;
}

void PTSolid2UltraMesh(PTSolid solid, UltraMesh& mesh)
{
	PTStatus status = PV_STATUS_OK;
	PTDirectSolidQuery query = PV_ENTITY_NULL;
	PTNat32 numFaces = 0;
	PTNat32 numVertices = 0;
	PTNat32 numEdges = 0;


	status |= PFSolidQueryDirectIndexed(solid, PV_MESH_TRIANGLES, NULL, &query);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_FACES, PV_DATA_TYPE_SIGNED_INT32, &numFaces, 0);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_EDGES, PV_DATA_TYPE_SIGNED_INT32, &numEdges, 0);

	status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_NUM_VERTICES, PV_DATA_TYPE_SIGNED_INT32, &numVertices, 0);

	printf("Read solid with %d Faces, %d Vertices and %d Edges\n", numFaces, numVertices, numEdges);

	Eigen::Vector3d vertex;
	Eigen::Vector3i face;
	double newV[3];

	for (int idx = 0; idx < int(numVertices); idx++)
	{
		status |= PFDirectSolidQuerySingle(query, PV_DSQUERY_VERTEX_COORDS, PV_DATA_TYPE_DOUBLE, &vertex, idx);
		for (int i = 0; i < 3; i++)
			newV[i] = vertex[i];
        UltraVertex newVertex = UltraVertex(Eigen::Vector3d(newV));
        newVertex.m_index = mesh.m_vertices.size();
		mesh.m_vertices.push_back(newVertex);
	}

	for (int idx = 0; idx < int(numFaces); idx++)
	{
		PFDirectSolidQuerySingle(query, PV_DSQUERY_FACE_VERTICES, PV_DATA_TYPE_UNSIGNED_INT32, &face, idx);
		Eigen::Vector3d p;
		Eigen::Vector3d pp[3];
		for (int i = 0; i < 3; i++)
		{
			p = mesh.m_vertices[face[i]].Position();
			for (int j = 0; j < 3; j++)
				pp[i][j] = p[j];
		}
		UltraFace newFace = UltraFace(face[0], face[1], face[2]);
		mesh.m_faces.push_back(newFace);
	}
	PFDirectSolidQueryDestroy(query);
    PFEntityGetBoundsProperty(solid, PV_SOLID_PROP_BOUNDS, mesh.m_bounds);

}


int main(int argc, char* argv[])

{
	//std::wstring fileName = L"c:\\parts\\castor\\Spiral_20480.stl";
    //std::wstring fileName = L"c:\\parts\\industrial\\bracket.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\Remeshed\\73986 LEVER_curve_sensitive.stl";
    //std::wstring fileName = L"C:\\Parts\\Industrial\\Rocker Cover.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\Coplanar\\coplanar_mesh1.stl";
    //std::wstring fileName = L"C:\\Parts\\Castor\\3dcross.stl";
    std::wstring fileName = L"C:\\Parts\\Castor\\gauges.stl";
    //std::wstring fileName = L"c:/temp/!hole.stl";

	PTInitialiseOpts initialise_options;
	PTEnvironment env = PV_ENTITY_NULL;
	PTEnvironmentOpts env_options;
	HWND window;
	PTDrawable drawable;
	PTWorld world;
	PTViewport vp;
	PTPoint vp_from = { 1000.0, 1000.0, 1000.0 };
	PTPoint vp_to = { 0.0, 0.0, 0.0 };
	PTVector vp_up = { 0.0, 0.0, 1.0 };

	double modelColor[3] = { 0.8, 0.8, 0.0 };
	double modelEdgeColor[3] = { 0.1, 0.1, 0.1 };
	PTNat32 modelTransparency = 50;
	bool modelViewEdges = false;
	bool modelView = true;
	double modifiedModelColor[3] = { 0.8, 0.8, 1.0 };
	double modifiedModelEdgeColor[3] = { 0.1, 0.1, 0.1 };
	PTNat32 modifiedModelTransparency = 50;
	bool modifiedModelViewEdges = true;
	bool modifiedModelView = true;

	double bgBottomColor[3] = { 0.2, 0.4, 0.6 };
	double bgTopColor[3] = { 0.1, 0.2, 0.3 };
	bool renderEdges = true;
	PTRenderStyle render_style;
	PTPolygonStyle poly_style;
	PTEdgeStyle edge_style;

	PTStatus status;


	PMInitInitialiseOpts(&initialise_options);
	status = PFInitialise(PV_LICENSE, &initialise_options);

#if JOURNAL_DEBUG
	PTStream journal_stream = PV_ENTITY_NULL;
	/* For debug builds journaling should be enabled for support  */
	/* Set journal file stream directly after initialising        */
	/* Polygonica                                                 */
	status = PFStreamFileOpen(PTPointer("c:\\temp\\PG_journal_ColorMeshSimplifier_2020_03_20_1.pgj"), PV_FILE_WRITE,
		NULL, &journal_stream);
	if (status == PV_STATUS_OK)
	{
		printf("Failed to start debug journal\n");
	}
	PFEntitySetEnumProperty(PV_ENTITY_NULL, PV_GLOBAL_PROP_JOURNAL_LEVEL, PV_JOURNAL_LEVEL_VERBOSE);
	PFEntitySetEntityProperty(PV_ENTITY_NULL, PV_GLOBAL_PROP_JOURNAL_STREAM, journal_stream);
#endif

	PMInitEnvironmentOpts(&env_options);
	status += PFEnvironmentCreate(&env_options, &env);

	window = PgWindowCreate(PTString("UltraMesh"), 600, 50, 1200, 800);

	status += PFDrawableCreate(env, window, NULL, &drawable);
	status += PFWorldCreate(env, NULL, &world);

	status += PFViewportCreate(world, &vp);

	PgWindowRegister(window, drawable, vp);
	PFEntitySetEnumProperty(vp, PV_VP_PROP_BG_MODE, PV_VP_BG_GRADUATED);
	PFEntitySetColourProperty(vp, PV_VP_PROP_BG_TOP_COL, PV_COLOUR_DOUBLE_RGB_ARRAY, bgTopColor);
	PFEntitySetColourProperty(vp, PV_VP_PROP_BG_BOTTOM_COL, PV_COLOUR_DOUBLE_RGB_ARRAY, bgBottomColor);

	status += PFDrawableRender(drawable, vp, PV_RENDER_MODE_BACKGROUND);

	printf("Reading file: %ls\n", fileName.c_str());

	PTSolid model = PV_ENTITY_NULL;
	PTSolid modifiedModel = PV_ENTITY_NULL;

	UltraMesh ultraMesh = UltraMesh();
	model = ReadSTL(env, fileName);
    if (REMESH)
    {
        PTSolidRemeshOpts initSolidRemeshOpts;
        PMInitSolidRemeshOpts(&initSolidRemeshOpts);
        initSolidRemeshOpts.keep_sharp_features = TRUE;
        initSolidRemeshOpts.remesh_limits = PV_REMESH_LIMIT_EDGE_LENGTH;
        initSolidRemeshOpts.keep_boundaries = TRUE;
        initSolidRemeshOpts.curvature_sensitive_remeshing = TRUE;
        initSolidRemeshOpts.max_edge_length = MINIMAL_WALL_THICKNESS ;
        initSolidRemeshOpts.edge_length = MINIMAL_WALL_THICKNESS;
        status += PFSolidRemesh(model, &initSolidRemeshOpts);
    }
	printf("Building mesh..\n");
	PTSolid2UltraMesh(model, ultraMesh);
	ultraMesh.CalcFaces();
	ultraMesh.MapEdges();
	ultraMesh.CalcNormals(false);
    ultraMesh.CalcCurvature();
	printf("Calculating buckets..\n");
	ultraMesh.CalcBuckets();
	int rays = 1;
	printf("Tracing %d rays...  ", rays);
	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 0; i < rays; i++)
	{
		Eigen::Vector3d origin = { (rand() / RAND_MAX - 0.5) * 100, 
			(rand() / RAND_MAX - 0.5) * 100, 
			(rand() / RAND_MAX - 0.5) * 100 };
		Eigen::Vector3d direction = { (rand() / RAND_MAX - 0.5) * 2, 
			 (rand() / RAND_MAX - 0.5) * 2,
			 (rand() / RAND_MAX - 0.5) * 2 };
		ultraMesh.IntersectWithRay(origin, direction);
	}
    UltraMesh modelMesh = ultraMesh;
    
    double halfThickness = MINIMAL_WALL_THICKNESS / 2.0;

    ultraMesh.OffsetBySkeleton(-halfThickness);
    //ultraMesh.Smooth();
    modelMesh.CalcThickness(ultraMesh);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Completed in %3.3f seconds. \n", TIME_INTERVAL(end, start));
    modelMesh.CalcColors(MINIMAL_WALL_THICKNESS * 0.9 / 2, MINIMAL_WALL_THICKNESS *1.1 / 2);
    modelMesh.SaveAsVRML(L"c:/temp/!thickness.wrl");
    ultraMesh.SaveAsVRML(L"c:/temp/!skeleton.wrl");

	modifiedModel = UltraMesh2PTSolid(ultraMesh, env);


	PTBounds modelBounds;

	PFEntityGetBoundsProperty(model, PV_SOLID_PROP_BOUNDS, modelBounds);


	PTWorldEntity modelEntity, modifiedModelEntity;

	status += PFWorldAddEntity(world, model, &modelEntity);
	status += PFWorldAddEntity(world, modifiedModel, &modifiedModelEntity);

	vp_to[0] = (modelBounds[0] + modelBounds[1]) / 2.0;
	vp_to[1] = (modelBounds[2] + modelBounds[3]) / 2.0;
	vp_to[2] = (modelBounds[4] + modelBounds[5]) / 2.0;
	vp_from[0] = vp_to[0];
	vp_from[1] = vp_to[1];
	vp_from[2] = vp_to[2] + modelBounds[5] - modelBounds[4];

	status += PFViewportSetPinhole(vp, &vp_from[0], &vp_to[0], &vp_up[0], PV_PROJ_ORTHOGRAPHIC, 500.0);
	status += PFViewportFit(vp, modelBounds);


	status += PFRenderStyleCreate(env, &render_style);
	poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
	edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
	PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modelColor);
	PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, modelTransparency);
	PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modelEdgeColor);
	PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, modelViewEdges);
	PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
	PFEntitySetEntityProperty(modelEntity, PV_WENTITY_PROP_STYLE, render_style);
	PFEntitySetBooleanProperty(modelEntity, PV_WENTITY_PROP_VISIBLE, modelView);
	status += PFRenderStyleDestroy(render_style);

	status += PFRenderStyleCreate(env, &render_style);
	poly_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_POLYGON_STYLE);
	edge_style = PFEntityGetEntityProperty(render_style, PV_RSTYLE_PROP_EDGE_STYLE);
	PFEntitySetColourProperty(poly_style, PV_PSTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modifiedModelColor);
	PFEntitySetNat32Property(poly_style, PV_PSTYLE_PROP_TRANSPARENCY, modifiedModelTransparency);
	PFEntitySetColourProperty(edge_style, PV_ESTYLE_PROP_COLOUR, PV_COLOUR_DOUBLE_RGB_ARRAY, modifiedModelEdgeColor);
	PFEntitySetBooleanProperty(render_style, PV_RSTYLE_PROP_RENDER_EDGES, modifiedModelViewEdges);
	PFEntitySetDoubleProperty(render_style, PV_RSTYLE_PROP_EDGE_ANGLE, 0.0);
	PFEntitySetEntityProperty(modifiedModelEntity, PV_WENTITY_PROP_STYLE, render_style);
	PFEntitySetBooleanProperty(modifiedModelEntity, PV_WENTITY_PROP_VISIBLE, modifiedModelView);
	status += PFRenderStyleDestroy(render_style);




	/* Adjust viewport to fit solid and render */
	status += PFDrawableRender(drawable, vp, PV_RENDER_MODE_SOLID);

	PROMPT((char*)"Press any key to terminate.\n");

	PFTerminate();

	return 0;
}