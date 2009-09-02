/*
*  Lembrete: at� o mudar o modelo 3D: X � o depth (o pr�prio X) e Y � a proje��o.
*
*
* A cada itera��o, os valores do depth e da proje��o s�o em rela��o a proje��o antiga.
*
*
*
* Trablhar tendo como centro de coordenadas a �ltima pos���o da c�mera. A entrada � um vetor
* de transla��o j� tranformado para esse sistema, e um �ngulo de diferen�a de inclina��o 
* entre as posi��es da camera.
*/
package br.com.r4j.research.image.sequence.featurematch.vline;

import java.awt.Color;
import java.awt.image.BufferedImage;

import org.apache.commons.logging.Log;
import org.apache.commons.logging.LogFactory;

import JSci.maths.AbstractDoubleSquareMatrix;
import br.com.r4j.commons.util.ColorSequence;
import br.com.r4j.commons.util.ImageUtil;
import br.com.r4j.image.operation.threebandpacked.ThreeBandPackedUtil;
import br.com.r4j.research.RigidBodyTranformation2D;
import br.com.r4j.research.image.sequence.CameraModel;
import br.com.r4j.research.vline.LineSegmentCollection;
import br.com.r4j.research.vline.MatchVLineMap;
import br.com.r4j.research.vline.VLine;
import br.com.r4j.research.vline.VLineMap;
import br.com.r4j.research.vline.VLinePerfil;
import br.com.r4j.research.vline.VLineProj;
import br.com.r4j.robosim.Pose2D;
import br.com.r4j.robosim.WorldMap;



/**
 * Chamado de tempos em tempos para atualizar a imagem.
 *
 *
 * Deve estar preparado ap�s cada chamada para oferecer:
 *	- As estimativas das linhas j� existentes com suas medi��es.
 *	- As estimativas de linhas novas encontradas. J� deve ter aparecido em
 *    duas imagens e ter calculado estimativa inicial.
 *	- As estimativas das linhas qeu foram perdidas. Talvez n�o ...
 *
 *
 *  Algor�tmo b�sico:
 *	- Limpar a imagem com objetivo de deixar apenas as retas.
 *	- Aplicar Hough para extrair as retinhas
 *	- Tentar casar as retas com estimativas anteriores:
 *		- Pegar as estimativas ordendas por u.
 *		- Ordenar as retas encontradas por u.
 *		- Considerando a zona de erro da proje��o anterior, da proje��o atual 
 *		e da estimativa do movimento, buscar por casamentos entre as imagens
 *		atrav�s de uma busca janelada ordenada. Para cada poss�vel casamento,
 *		calcular o perfil da reta caso n�o calculado e comparar com o antigo.
 *		escolher o melhor casamento. POde-se melhorar o algor�tmo esperando
 *		a conclus�o de todos os casamentos e ir escolhendo os melhores 
 *		globalemnte, considerando as diferen�as entre os casamentos para um reta
 *		em rela��o as demias 'competidoras' por um casamento.
 *	- Se a reta casada n�o for v�lida, criar uma primeira estimativa para a mesma
 *	utilizando as restri��es epipolares.
 *	- As linhas n�o casadas s�o marcadas como n�o vis�veis, e as medi��es n�o
 * casadas s�o adicionadas como retas n�o v�lidas.
 *
 *
 *
 *  Defini��es de opera��o d VLineMap
 *   - uma vez uma linha entrada, a linha manter� o mesmo mapIdx at� ser eliminada.
 *   - uma vez eliminada, o �ndice mapIdx ainda ser� v�lido, podendo ser usado por
 *     outra nova linha, por�m deve ficar por uma itera��o como inv�lida.
 *
 *
 *
 *
 *
 */
public class Matcher
{
	private static Log log = LogFactory.getLog(Matcher.class.getName());
	private static Log logLineProj = LogFactory.getLog("line_proj");
	private static Log logTime = LogFactory.getLog("time");

	
	///// Vari�veis com a descri��o do sistema
	//
	private CameraModel camModel = null;

	///// Vari�veis com o estado do sistema
	//
	private MatchVLineMap lineMap = null;

	private WorldMap worldMap = null;

	private ExtractionPhaseMatcher extractionPhase = null;
	private DifferenceSimpleMatchingPhaseMatcher matchingPhase = null;


	public Matcher()
	{
		extractionPhase = new ExtractionPhaseMatcher();
		matchingPhase = new DifferenceSimpleMatchingPhaseMatcher();

		lineMap = new MatchVLineMap();

//		extractionPhase.setLineMap(lineMap);
		matchingPhase.setLineMap(lineMap);
		matchingPhase.setWorldMap(worldMap);

	}


	public void setCameraModel(CameraModel camModel)
	{
		this.camModel = camModel;
//		extractionPhase.setCameraModel(camModel);
		matchingPhase.setCameraModel(camModel);
	}

	public void setWorldMap(WorldMap worldMap)
	{
		this.worldMap = worldMap;
		matchingPhase.setWorldMap(worldMap);
	}

	public void setSimpleMatch(boolean bVal)	{matchingPhase.setSimpleMatch(bVal);}


	public void setUseWindow(boolean bVal)
	{
		matchingPhase.setUseWindow(bVal);
	}

	public void setMinDepth(int minDepth)
	{
		matchingPhase.setMinDepth(minDepth);
	}

	public void setHashVectorTreshold(double hashVectorTreshold)
	{
		matchingPhase.setHashVectorTreshold(hashVectorTreshold);
	}

	public void setUseDirectionWindow(boolean hashVectorTreshold)
	{
		matchingPhase.setUseDirectionWindow(hashVectorTreshold);
	}

	public void setDirectWindowSize(int hashVectorTreshold)
	{
		matchingPhase.setDirectWindowSize(hashVectorTreshold);
	}


	public void setHashVectorSideBandSize(int hashVectorSideBandSize)
	{
		VLinePerfil.PERFIL_SIZE = 2 * hashVectorSideBandSize + 1;
	}

	public void setHashVectorAttenuationMax(double hashVectorAttenuationMax)
	{
		VLinePerfil.MAX_ATTENUATION = hashVectorAttenuationMax;
	}

	public void setHashVectorHistorySize(int hashHist)
	{
		VLinePerfil.PERFIL_HISTORICO = hashHist;
	}

	public void setCompensateIllumination(boolean hashHist)
	{
		VLinePerfil.APPLY_HISTOGRAM = hashHist;
	}


	/**
	 * Pega o mapa atual de estados
	 */
	public VLineMap getStateMap()
	{
		return lineMap;
	}


	/**
	 * M�todo principal, onde a imagem � processada.
	 *
	 * poseEstimate j� esperado invertido, e o erro associado a ele deve ser o erro
	 * global.
	 *
	 * poseEstimateMove j� esperado invertido, e o erro associado a ele deve ser o
	 * erro apenas do �ltimo deslocamento.
	 *
	 */
	private int [] arrayOutDebug = null;
	private int itCount = 0; // de-de-debug!
	public void update(int [] arrayImg, int imgWidth, int imgHeight, RigidBodyTranformation2D cameraTrafo, AbstractDoubleSquareMatrix cameraPoseCovar, Pose2D cameraMoveEstimate, AbstractDoubleSquareMatrix  cameraMoveCovar)
	{
		if (log.isDebugEnabled())
		{
			if (arrayOutDebug == null || arrayOutDebug.length < arrayImg.length)
				arrayOutDebug = new int [arrayImg.length];
			if (itCount < 10)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImg,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "ori_00" + itCount + 
				".jpg");
			else if (itCount < 100)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImg,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "ori_0" + itCount + 
				".jpg");
			else
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayImg,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "ori_" + itCount + 
				".jpg");
		}

		long tmTm = 0, start_t = System.currentTimeMillis();
		extractionPhase.setItCount(itCount);
		matchingPhase.setItCount(itCount);

		log.debug("itCount = " + itCount);
		logTime.debug("itCount = " + itCount);
		log.debug("cameraTrafo = " + cameraTrafo);
		log.debug("cameraMoveEstimate = " + cameraMoveEstimate);

		tmTm = System.currentTimeMillis();
		LineExtrationResult newLineProjs = extractionPhase.lineExtraction(arrayImg, imgWidth, imgHeight);
		logTime.debug("extraction: " + (System.currentTimeMillis() - tmTm));

		tmTm = System.currentTimeMillis();
		LineMatchingResult matchngResult = matchingPhase.lineMatching(arrayImg, imgWidth, imgHeight, cameraTrafo, cameraPoseCovar, cameraMoveEstimate, cameraMoveCovar, newLineProjs, true);
		logTime.debug("matching: " + (System.currentTimeMillis() - tmTm));
		
		logTime.debug("alg geral: " + (System.currentTimeMillis() - start_t));

		/////
		// Gera a sa�da caso necess�rio
		if (log.isDebugEnabled())
		{
			br.com.r4j.commons.util.Arrays.arrayCopy(arrayImg, arrayOutDebug);

			for (int i = 0; i < newLineProjs.arrayProjMeasures.length; i++)
			{
				boolean bIsNewLine = false;
				VLineProj lineProj = newLineProjs.arrayProjMeasures[i];
				LineSegmentCollection lCol = lineProj.getSegment();
				VLine line = matchngResult.arrayProjMeasuresLines[i];
				Color clr = null;
				clr = lineMap.getDebugColor(line);
				if (clr == null)
				{
					bIsNewLine = true;
					clr = ColorSequence.getColor(lineMap.nextColorIdx++, 0);
//					clr = Color.white;
					lineMap.setDebugColor(line, clr);
				}
/*
				else if (clr == Color.white)
				{
					clr = ColorSequence.getNonWhiteNonBlackColor(lineMap.nextColorIdx++, 0);
					lineMap.setDebugColor(line, clr);
				}
//*/
				int clrVal = ThreeBandPackedUtil.getPackedPixel(clr);
/*
				int u = (int) lineProj.getU();
				if (u >= imgWidth) 
					u = imgWidth - (u - imgWidth) - 2;
				if (u < 0) 
					u = -u;


				for (int j = 0; j < imgHeight; j++)
					arrayOutDebug[j*imgWidth + u] = clrVal;
				for (int j = 0; j < imgHeight; j++)
					arrayOutDebug[j*imgWidth + u] = clrVal;
//*/
				for (int v = lCol.yIni; v <= lCol.yEnd; v++)
				{
					int u = (int) (lCol.mModel*v + lCol.xModel);
					if (u < 0) u = 0;
					if (u >= imgWidth) u = imgWidth - 1;
					arrayOutDebug[v*imgWidth + u] = clrVal;
				}

				int lineMark = line.mapIdx*10%(lCol.yEnd - lCol.yIni) + lCol.yIni;
				int uMid = (int) (lCol.mModel*lineMark + lCol.xModel);
				if (bIsNewLine) for (int j = uMid - 3; j < uMid + 4; j++)
				{
					if (j >= 0 && j < imgWidth)
						arrayOutDebug[lineMark*imgWidth + j] = 0xFFFFFF;

				}
				else for (int j = uMid - 3; j < uMid + 4; j++)
				{
					if (j >= 0 && j < imgWidth)
						arrayOutDebug[lineMark*imgWidth + j] = clrVal;

				}
/*
				if (u > 0) for (int j = 0; j < imgHeight; j++)
					arrayOutDebug[j*imgWidth + u - 1] = clrVal;
				if (u < imgWidth) for (int j = 0; j < imgHeight; j++)
					arrayOutDebug[j*imgWidth + u + 1] = clrVal;
//*/
			}
			if (itCount < 10)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "debA_00" + itCount + 
				".jpg");
			else if (itCount < 100)
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "debA_0" + itCount + 
				".jpg");
			else
				ImageUtil.saveImageJPEG(ImageUtil.createBufferedImage(arrayOutDebug,imgWidth,imgHeight,BufferedImage.TYPE_INT_RGB), "debA_" + itCount + 
				".jpg");
/*
			Iterator itMaximas = listMaximas.iterator();
			while (itMaximas.hasNext())
			{
				Point pt = (Point) itMaximas.next();
				int u = pt.x;
				for (int i = 0; i < imgHeight; i++)
					arrayOutDebug[i*imgWidth + u] |= 0xFF0000;
			}
//*/
		}
		itCount++;
	}
}





