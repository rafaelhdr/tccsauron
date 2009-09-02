select
	proj.prj_id prj_id,
	proj.co co,
	proj.unique_name unique_name,
	proj.nome nome,
	proj.fase fase,
	proj.fase_name fase_name,
	proj.categoria_name categoria_name,
	proj.coordenador coordenador,
	proj.prj_start_date prim_start_date,
	proj.prj_end_date prim_end_date
from
	(select 
		current_date anoref
	  from 
		dual
	) t_anoref,
	(select
		p.id prj_id,
		pud.ecb_co co,
		pud.ecb_status,
		p.name nome,
		p.unique_name unique_name,
		pud.cr_dynamic cr,
		pud.ecb_codigoorcamento cod_orcamento,
		pud.ecb_codigoiniciativa cod_iniciativa,

		pud.faseinvestestagio fase,
		caps3.name fase_name,

		prj.prstart prj_start_date,
		prj.prfinish prj_end_date,
		prj.progress prj_expended,
		prj.prstart prj_base_start_date,
		prj.prfinish prj_base_end_date,
		painelData.relatRealizado,
		painelData.statusRisco,
		painelData.statusTempo,
		painelData.statusOrc,
		painelData.statusDataRef,
		painelData.capturaAprov,
		painelData.projConluido,
		painelData.statusgeral,
		pud.ecb_braskemmais ecb_braskemmais,
		pud.ecb_pn ecb_pn,
		painelData.coordenador,

		pud.ecb_categoria categoria_id,
		caps2.name categoria_name,
		NULL faseinvest,
		painelData.coordorggestorname gestorun
	from
		srm_projects p
			left join odf_ca_ecb_preaprovado pre 
			on p.id = pre.odf_parent_id 
			and pre.anobase = to_number(to_char(current_date, 'yyyy'))

			left join ecbEspelhoProjectServer pServer 
			on p.id = pServer.prj_id

			join ecbPainelControleConsolidado painelData
			on p.id = painelData .pid

			join odf_ca_project pud 
				left join cmn_lookups lookups2 
				join cmn_captions_nls caps2 
				on caps2.PK_ID = lookups2.id and 
				caps2.language_code = 'en' and 
				caps2.table_name = 'CMN_LOOKUPS' 
			on lookups2.lookup_type = 'ECB_CATEGORIA' and 
			pud.ecb_categoria = lookups2.lookup_enum

			left join cmn_lookups lookups3 
				join cmn_captions_nls caps3 
				on caps3.PK_ID = lookups3.id and 
				caps3.language_code = 'en' and 
				caps3.table_name = 'CMN_LOOKUPS' 
			on lookups3.lookup_type = 'ECB_FASEINVESTIMENTO_ESTAGIO' and 
			pud.faseinvestestagio = lookups3.lookup_enum

			on p.id = pud.id

			join prj_projects prj 
			left join cmn_sec_users c on prj.manager_id = c.id
			on p.id = prj.prid

			join prj_obs_associations assocPrj
				join nbi_dim_obs_flat flatPrj
				on assocPrj.unit_id = flatPrj.child_obs_unit_id and
				assocPrj.table_name ='SRM_PROJECTS' and
				flatPrj.parent_obs_unit_id = 5000172
			on p.id = assocPrj.record_id
	where
		p.is_active = 1 and 
		(5049401 = getResourceID4RoleId4Proj(5038144, pud.id))
	) proj
order by
	proj.prj_id









































select
	proj.prj_id prj_id,
	proj.co co,
	proj.unique_name unique_name,
	proj.nome nome,
	proj.fase fase,
	proj.fase_name fase_name,
	proj.categoria_name categoria_name,
	proj.prj_start_date prim_start_date,
	proj.prj_end_date prim_end_date
from
	(select 
		current_date anoref
	  from 
		dual
	) t_anoref,
	(select
		p.id prj_id,
		pud.ecb_co co,
		pud.ecb_status,
		p.name nome,
		p.unique_name unique_name,
		pud.cr_dynamic cr,
		pud.ecb_codigoorcamento cod_orcamento,
		pud.ecb_codigoiniciativa cod_iniciativa,

		pud.faseinvestestagio fase,
		caps3.name fase_name,

		prj.prstart prj_start_date,
		prj.prfinish prj_end_date,
		prj.progress prj_expended,
		prj.prstart prj_base_start_date,
		prj.prfinish prj_base_end_date,
		pud.ecb_braskemmais ecb_braskemmais,
		pud.ecb_pn ecb_pn,

		pud.ecb_categoria categoria_id,
		caps2.name categoria_name,
		NULL faseinvest
	from
		srm_projects p
			left join odf_ca_ecb_preaprovado pre 
			on p.id = pre.odf_parent_id 
			and pre.anobase = to_number(to_char(current_date, 'yyyy'))

			left join ecbEspelhoProjectServer pServer 
			on p.id = pServer.prj_id

			join odf_ca_project pud 
				left join cmn_lookups lookups2 
					join cmn_captions_nls caps2 
					on caps2.PK_ID = lookups2.id and 
					caps2.language_code = 'en' and 
					caps2.table_name = 'CMN_LOOKUPS' 
				on lookups2.lookup_type = 'ECB_CATEGORIA' and 
				pud.ecb_categoria = lookups2.lookup_enum

				left join cmn_lookups lookups3 
					join cmn_captions_nls caps3 
					on caps3.PK_ID = lookups3.id and 
					caps3.language_code = 'en' and 
					caps3.table_name = 'CMN_LOOKUPS' 
				on lookups3.lookup_type = 'ECB_FASEINVESTIMENTO_ESTAGIO' and 
				pud.faseinvestestagio = lookups3.lookup_enum
			on p.id = pud.id

			join prj_projects prj 
				left join cmn_sec_users c on prj.manager_id = c.id
				on p.id = prj.prid
	where
		p.is_active = 1 and 
		p.id = 5002018 and
		(5049401 = getResourceID4RoleId4Proj(5038144, pud.id))
	) proj
order by
	proj.prj_id




	p.is_active = 1 and 
	p.id = 5002018
	(5049401 = getResourceID4RoleId4Proj(5038144, pud.id))

















create or replace function ecbCompResID4RoleId4Proj(resource_id in number, role_id in number, prjId in number) return NUMBER
as

countPeople number;

begin
	SELECT
		count(srmResPerson.id) into countPeople
	FROM
		(srm_resources srmResRole
			JOIN (prj_resources prjResRole 
				JOIN (prteam prteam 
					JOIN (prj_resources prjResPerson 
						JOIN srm_resources srmResPerson 
						ON srmResPerson.id = prjResPerson.prid and srmResPerson.id = resource_id)
					ON prteam.prresourceid = prjResPerson.prid)
				ON prteam.prroleid = prjResRole.prid AND prteam.prprojectid = prjId)
			ON srmResRole.id = prjResRole.prid and srmResRole.id = role_id);
	return countPeople;
end;





select
	srmResPerson.id,
	srmResPerson.first_name,
	srmResPerson.last_name,
	srmResPerson.unique_name,
	srmResRole.id,
	srmResRole.unique_name,
	prteam.prprojectid,
	getResourceID4RoleId4Proj(srmResRole.id, prteam.prprojectid),
	ecbCompResID4RoleId4Proj(srmResPerson.id, srmResRole.id, prteam.prprojectid)
from
	(srm_resources srmResRole 
		JOIN (prj_resources prjResRole 
			JOIN (prteam prteam 
				JOIN (prj_resources prjResPerson 
					JOIN srm_resources srmResPerson 
					ON srmResPerson.id = prjResPerson.prid)
				ON prteam.prresourceid = prjResPerson.prid)
			ON prteam.prroleid = prjResRole.prid)
		ON srmResRole.id = prjResRole.prid)
where
	srmResPerson.first_name like 'Angel%'
order by
	prteam.prprojectid,
	srmResPerson.first_name,
	srmResPerson.last_name,
	srmResPerson.unique_name



select
	id
from
	odf_ca_project
where
	ecb_co = '101208'
